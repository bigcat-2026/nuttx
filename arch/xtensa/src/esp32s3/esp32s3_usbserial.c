/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_usbserial.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <debug.h>
#include <sched.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/tioctl.h>
#include <arch/irq.h>

#include "xtensa.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_usb_serial_jtag.h"
#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_interrupt_core0.h"
#ifdef CONFIG_SMP
#  include "hardware/esp32s3_interrupt_core1.h"
#endif

#include "esp32s3_config.h"
#include "esp32s3_irq.h"

/****************************************************************************
 * Pre-processor Macros
 ****************************************************************************/

/* The hardware buffer has a fixed size of 64 bytes */

#define ESP32S3_USBCDC_BUFFERSIZE 64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_priv_s
{
  const uint8_t  periph;        /* peripheral ID */
  const uint8_t  irq;           /* IRQ number assigned to the peripheral */
  int            cpu;           /* CPU id */
  int            cpuint;        /* CPU interrupt assigned */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32s3_interrupt(int irq, void *context, void *arg);
static int esp32s3_usbserial_poll_thread(int argc, char *argv[]);
static int esp32s3_usbserial_diag_thread(int argc, char *argv[]);
static int esp32s3_usbserial_heartbeat_thread(int argc, char *argv[]);
static void esp32s3_usbserial_start_poll(const char *tag);
static void esp32s3_usbserial_force_pad(const char *tag);
static void esp32s3_usbserial_diag_once(int index);
static uint32_t esp32s3_get_intenable(void);
void esp32s3_usbserial_write(char ch);
bool esp32s3_usbserial_trywrite(char ch);
void esp32s3_usbserial_debug_dump(const char *tag);
void esp32s3_usbserial_debug_state(const char *tag);
void esp32s3_usbserial_debug_restore(void);
void esp32s3_usbserial_debug_reattach_irq(void);
void esp32s3_usbserial_debug_reattach_irq_cpu(int cpu);
void esp32s3_usbserial_recover_rx(void);
void esp32s3_usbserial_reopen(void);
void esp32s3_usbserial_recover_level(int level);
void esp32s3_usbserial_start_diag(void);
void esp32s3_usbserial_start_heartbeat(void);
int esp32s3_usbserial_raw_probe(int seconds, uint32_t *last_ep1conf,
                                uint32_t *last_out1,
                                uint32_t *last_intst);

/* Serial driver methods */

static int  esp32s3_setup(struct uart_dev_s *dev);
static void esp32s3_shutdown(struct uart_dev_s *dev);
static int  esp32s3_attach(struct uart_dev_s *dev);
static void esp32s3_detach(struct uart_dev_s *dev);
static void esp32s3_txint(struct uart_dev_s *dev, bool enable);
static void esp32s3_rxint(struct uart_dev_s *dev, bool enable);
static bool esp32s3_rxavailable(struct uart_dev_s *dev);
static bool esp32s3_txready(struct uart_dev_s *dev);
static bool esp32s3_txempty(struct uart_dev_s *dev);
static void esp32s3_send(struct uart_dev_s *dev, int ch);
static int  esp32s3_receive(struct uart_dev_s *dev, unsigned int *status);
static int  esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_rxbuffer[ESP32S3_USBCDC_BUFFERSIZE];
static char g_txbuffer[ESP32S3_USBCDC_BUFFERSIZE];

static struct esp32s3_priv_s g_usbserial_priv =
{
  .periph = ESP32S3_PERIPH_USB_DEVICE,
  .irq    = ESP32S3_IRQ_USB_DEVICE,
  .cpu    = 0,
  .cpuint = -ENOMEM,
};

static struct uart_ops_s g_uart_ops =
{
  .setup       = esp32s3_setup,
  .shutdown    = esp32s3_shutdown,
  .attach      = esp32s3_attach,
  .detach      = esp32s3_detach,
  .txint       = esp32s3_txint,
  .rxint       = esp32s3_rxint,
  .rxavailable = esp32s3_rxavailable,
  .txready     = esp32s3_txready,
  .txempty     = esp32s3_txempty,
  .send        = esp32s3_send,
  .receive     = esp32s3_receive,
  .ioctl       = esp32s3_ioctl,
};

static volatile bool g_usbserial_poll_started;
static volatile uint32_t g_usbserial_poll_started_seen;
static volatile uint32_t g_usbserial_poll_heartbeat_seen;

static struct
{
  bool     valid;
  uint32_t conf0;
  uint32_t mem;
  uint32_t misc;
  uint32_t intena;
} g_usbserial_snapshot;

/****************************************************************************
 * Public Data
 ****************************************************************************/

uart_dev_t g_uart_usbserial =
{
  .isconsole = true,
  .recv      =
    {
      .size    = ESP32S3_USBCDC_BUFFERSIZE,
      .buffer  = g_rxbuffer,
    },
  .xmit      =
    {
      .size    = ESP32S3_USBCDC_BUFFERSIZE,
      .buffer  = g_txbuffer,
    },
  .ops       = &g_uart_ops,
  .priv      = &g_usbserial_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_interrupt
 *
 * Description:
 *   This is the common UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int esp32s3_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_INT_ST_REG);

  /* Send buffer has room and can accept new data. */

  if (regval & USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ST)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      uart_xmitchars(dev);
    }

  /* Data from the host are available to read. */

  if (regval & USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ST)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      uart_recvchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32s3_usbserial_poll_thread
 ****************************************************************************/

static int esp32s3_usbserial_poll_thread(int argc, char *argv[])
{
  bool rx_seen = false;
  int loops = 0;
  int rx_logs = 0;

  g_usbserial_poll_started_seen = 1;

  while (true)
    {
      if ((loops & 0x3f) == 0)
        {
          modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                      USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
        }

      if (esp32s3_rxavailable(&g_uart_usbserial))
        {
          if (!rx_seen || rx_logs < 8)
            {
              rx_seen = true;
              rx_logs++;
            }

          putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
                   USB_SERIAL_JTAG_INT_CLR_REG);
          uart_recvchars(&g_uart_usbserial);
        }

      loops++;

      if (loops <= 20000 && (loops % 1000) == 0)
        {
          g_usbserial_poll_heartbeat_seen++;
        }

      usleep(500);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s3_usbserial_start_poll
 ****************************************************************************/

static void esp32s3_usbserial_start_poll(const char *tag)
{
  int pid;

  if (g_usbserial_poll_started)
    {
      return;
    }

  g_usbserial_poll_started = true;
  pid = kthread_create("usbserpoll", CONFIG_INIT_PRIORITY - 10, 1024,
                       esp32s3_usbserial_poll_thread, NULL);

#ifdef CONFIG_SMP
  if (pid > 0 && CONFIG_SMP_NCPUS > 1)
    {
      cpu_set_t cpuset;

      CPU_ZERO(&cpuset);
      CPU_SET(1, &cpuset);
      nxsched_set_affinity(pid, sizeof(cpuset), &cpuset);
    }
#endif
}

/****************************************************************************
 * Name: esp32s3_usbserial_diag_thread
 ****************************************************************************/

static int esp32s3_usbserial_diag_thread(int argc, char *argv[])
{
  int i;

  for (i = 0; i < 10; i++)
    {
      esp32s3_usbserial_diag_once(i);
      sleep(1);
    }

  return 0;
}

static void esp32s3_usbserial_diag_once(int index)
{
  printf("USB diag %d: cpu=%d ps=%08" PRIx32
         " cpuintena=%08" PRIx32 " intena=%08" PRIx32
         " intst=%08" PRIx32 " ep1conf=%08" PRIx32
         " out1=%08" PRIx32 " jfifo=%08" PRIx32
         " conf0=%08" PRIx32 " mem=%08" PRIx32
         " misc=%08" PRIx32 " rxavail=%d txready=%d\n",
         index, sched_getcpu(), xtensa_getps(), esp32s3_get_intenable(),
         getreg32(USB_SERIAL_JTAG_INT_ENA_REG),
         getreg32(USB_SERIAL_JTAG_INT_ST_REG),
         getreg32(USB_SERIAL_JTAG_EP1_CONF_REG),
         getreg32(USB_SERIAL_JTAG_OUT_EP1_ST_REG),
         getreg32(USB_SERIAL_JTAG_JFIFO_ST_REG),
         getreg32(USB_SERIAL_JTAG_CONF0_REG),
         getreg32(USB_SERIAL_JTAG_MEM_CONF_REG),
         getreg32(USB_SERIAL_JTAG_MISC_CONF_REG),
         esp32s3_rxavailable(&g_uart_usbserial) ? 1 : 0,
         esp32s3_txready(&g_uart_usbserial) ? 1 : 0);
  fflush(stdout);
}

void esp32s3_usbserial_diag_sync(int loops)
{
  volatile uint32_t spin;
  int i;

  if (loops < 1)
    {
      loops = 1;
    }

  for (i = 0; i < loops; i++)
    {
      esp32s3_usbserial_force_pad("diag-sync");
      esp32s3_usbserial_diag_once(i);

      for (spin = 0; spin < 250000; spin++)
        {
        }
    }
}

static uint32_t esp32s3_get_intenable(void)
{
  uint32_t intenable;

  __asm__ __volatile__
  (
    "rsr %0, INTENABLE" : "=r"(intenable)
  );

  return intenable;
}

void esp32s3_usbserial_start_diag(void)
{
  kthread_create("usbserdiag", CONFIG_INIT_PRIORITY - 5, 2048,
                 esp32s3_usbserial_diag_thread, NULL);
}

static int esp32s3_usbserial_heartbeat_thread(int argc, char *argv[])
{
  int i;

  for (i = 0; i < 8; i++)
    {
      esp32s3_usbserial_trywrite('.');
      esp32s3_usbserial_trywrite('0' + i);
      sleep(1);
    }

  esp32s3_usbserial_trywrite('\n');
  return 0;
}

/****************************************************************************
 * Name: esp32s3_usbserial_force_pad
 ****************************************************************************/

static void esp32s3_usbserial_force_pad(const char *tag)
{
  static int log_count;
  uint32_t before;
  uint32_t after;
  uint32_t rtc_usb;

  before = getreg32(USB_SERIAL_JTAG_CONF0_REG);
  after = before | USB_SERIAL_JTAG_USB_PAD_ENABLE |
          USB_SERIAL_JTAG_DP_PULLUP;
  after &= ~(USB_SERIAL_JTAG_DP_PULLDOWN |
             USB_SERIAL_JTAG_DM_PULLUP |
             USB_SERIAL_JTAG_DM_PULLDOWN);

  putreg32(after, USB_SERIAL_JTAG_CONF0_REG);

  rtc_usb = getreg32(RTC_CNTL_RTC_USB_CONF_REG);
  rtc_usb |= RTC_CNTL_SW_HW_USB_PHY_SEL |
             RTC_CNTL_USB_PAD_ENABLE_OVERRIDE |
             RTC_CNTL_USB_PAD_ENABLE |
             RTC_CNTL_USB_DP_PULLUP;
  rtc_usb &= ~(RTC_CNTL_SW_USB_PHY_SEL |
               RTC_CNTL_USB_DP_PULLDOWN |
               RTC_CNTL_USB_DM_PULLUP |
               RTC_CNTL_USB_DM_PULLDOWN);
  putreg32(rtc_usb, RTC_CNTL_RTC_USB_CONF_REG);

  putreg32((getreg32(USB_SERIAL_JTAG_MEM_CONF_REG) &
            ~USB_SERIAL_JTAG_USB_MEM_PD) |
           USB_SERIAL_JTAG_USB_MEM_CLK_EN,
           USB_SERIAL_JTAG_MEM_CONF_REG);
  putreg32(getreg32(USB_SERIAL_JTAG_MISC_CONF_REG) |
           USB_SERIAL_JTAG_CLK_EN,
           USB_SERIAL_JTAG_MISC_CONF_REG);

  modifyreg32(USB_SERIAL_JTAG_JFIFO_ST_REG, 0,
              USB_SERIAL_JTAG_OUT_FIFO_RESET);
  up_udelay(10);
  modifyreg32(USB_SERIAL_JTAG_JFIFO_ST_REG,
              USB_SERIAL_JTAG_OUT_FIFO_RESET, 0);

  putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR |
           USB_SERIAL_JTAG_OUT_EP1_ZERO_PAYLOAD_INT_CLR |
           USB_SERIAL_JTAG_PID_ERR_INT_CLR |
           USB_SERIAL_JTAG_CRC5_ERR_INT_CLR |
           USB_SERIAL_JTAG_CRC16_ERR_INT_CLR |
           USB_SERIAL_JTAG_STUFF_ERR_INT_CLR,
           USB_SERIAL_JTAG_INT_CLR_REG);
  modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
              USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);

  if (before != after && log_count < 4)
    {
      printf("usbserial: force-pad %s conf0 %08" PRIx32 "->%08" PRIx32
             "\n",
             tag, before, after);
      fflush(stdout);
      log_count++;
    }
}

void esp32s3_usbserial_start_heartbeat(void)
{
  kthread_create("usbserhb", CONFIG_INIT_PRIORITY - 5, 1024,
                 esp32s3_usbserial_heartbeat_thread, NULL);
}

/****************************************************************************
 * Name: esp32s3_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int esp32s3_setup(struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: esp32s3_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.
 *
 ****************************************************************************/

static void esp32s3_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: esp32s3_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void esp32s3_txint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA);
    }
  else
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG,
                  USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_ENA, 0);
    }
}

/****************************************************************************
 * Name: esp32s3_rxint
 *
 * Description:
 *   Call to enable or disable RXRDY interrupts
 *
 ****************************************************************************/

static void esp32s3_rxint(struct uart_dev_s *dev, bool enable)
{
  if (enable)
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
    }
  else
    {
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA, 0);
    }
}

/****************************************************************************
 * Name: esp32s3_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may
 *   operate in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int esp32s3_attach(struct uart_dev_s *dev)
{
  struct esp32s3_priv_s *priv = dev->priv;
  int ret;

  DEBUGASSERT(priv->cpuint == -ENOMEM);

  /* Try to attach the IRQ to a CPU int */

  priv->cpu = this_cpu();
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->periph,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      return priv->cpuint;
    }

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, esp32s3_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32s3_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void esp32s3_detach(struct uart_dev_s *dev)
{
  struct esp32s3_priv_s *priv = dev->priv;

  DEBUGASSERT(priv->cpuint != -ENOMEM);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  esp32s3_teardown_irq(priv->cpu, priv->periph, priv->cpuint);

  priv->cpuint = -ENOMEM;
}

/****************************************************************************
 * Name: esp32s3_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool esp32s3_rxavailable(struct uart_dev_s *dev)
{
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_EP1_CONF_REG);

  return regval & USB_SERIAL_JTAG_SERIAL_OUT_EP_DATA_AVAIL;
}

/****************************************************************************
 * Name: esp32s3_txempty
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp32s3_txempty(struct uart_dev_s *dev)
{
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_JFIFO_ST_REG);

  return regval & USB_SERIAL_JTAG_OUT_FIFO_EMPTY;
}

/****************************************************************************
 * Name: esp32s3_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (TXRDY)
 *
 ****************************************************************************/

static bool esp32s3_txready(struct uart_dev_s *dev)
{
  uint32_t regval;

  regval = getreg32(USB_SERIAL_JTAG_EP1_CONF_REG);

  return regval & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE;
}

/****************************************************************************
 * Name: esp32s3_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void esp32s3_send(struct uart_dev_s *dev, int ch)
{
  /* Write the character to the buffer. */

  putreg32(ch, USB_SERIAL_JTAG_EP1_REG);

  /* Flush the character out. */

  putreg32(USB_SERIAL_JTAG_WR_DONE, USB_SERIAL_JTAG_EP1_CONF_REG);
}

/****************************************************************************
 * Name: esp32s3_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character.
 *
 ****************************************************************************/

static int esp32s3_receive(struct uart_dev_s *dev, unsigned int *status)
{
  *status = 0;
  return getreg32(USB_SERIAL_JTAG_EP1_REG) & USB_SERIAL_JTAG_RDWR_BYTE;
}

/****************************************************************************
 * Name: esp32s3_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int esp32s3_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TERMIOS)
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
          }
        else
          {
            /* The USB Serial Console has fixed configuration of:
             *    9600 baudrate, no parity, 8 bits, 1 stopbit.
             */

            termiosp->c_cflag = CS8;
            cfsetispeed(termiosp, 9600);
          }
      }
      break;

    case TCSETS:
      ret = -ENOTTY;
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial.  Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp32s3_usbserial_write(char ch)
{
  while (!esp32s3_txready(&g_uart_usbserial));

  esp32s3_send(&g_uart_usbserial, ch);
}

/****************************************************************************
 * Name: esp32s3_usbserial_trywrite
 ****************************************************************************/

bool esp32s3_usbserial_trywrite(char ch)
{
  if (!esp32s3_txready(&g_uart_usbserial))
    {
      return false;
    }

  esp32s3_send(&g_uart_usbserial, ch);
  return true;
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_save
 ****************************************************************************/

void esp32s3_usbserial_debug_save(void)
{
  g_usbserial_snapshot.conf0  = getreg32(USB_SERIAL_JTAG_CONF0_REG);
  g_usbserial_snapshot.mem    = getreg32(USB_SERIAL_JTAG_MEM_CONF_REG);
  g_usbserial_snapshot.misc   = getreg32(USB_SERIAL_JTAG_MISC_CONF_REG);
  g_usbserial_snapshot.intena = getreg32(USB_SERIAL_JTAG_INT_ENA_REG);
  g_usbserial_snapshot.valid  = true;
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_restore
 ****************************************************************************/

void esp32s3_usbserial_debug_restore(void)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;
  uint32_t conf0;
  uint32_t mem;
  uint32_t misc;
  uint32_t intena;

  if (!g_usbserial_snapshot.valid)
    {
      return;
    }

  conf0 = getreg32(USB_SERIAL_JTAG_CONF0_REG) |
          g_usbserial_snapshot.conf0 |
          USB_SERIAL_JTAG_USB_PAD_ENABLE |
          USB_SERIAL_JTAG_DP_PULLUP;
  mem = (g_usbserial_snapshot.mem & ~USB_SERIAL_JTAG_USB_MEM_PD) |
        USB_SERIAL_JTAG_USB_MEM_CLK_EN;
  misc = g_usbserial_snapshot.misc | USB_SERIAL_JTAG_CLK_EN;
  intena = g_usbserial_snapshot.intena |
           USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA;
  putreg32(conf0, USB_SERIAL_JTAG_CONF0_REG);
  putreg32(mem, USB_SERIAL_JTAG_MEM_CONF_REG);
  putreg32(misc, USB_SERIAL_JTAG_MISC_CONF_REG);
  putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
           USB_SERIAL_JTAG_INT_CLR_REG);
  putreg32(intena, USB_SERIAL_JTAG_INT_ENA_REG);
  up_enable_irq(priv->irq);
  esp32s3_usbserial_force_pad("restore");
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_reattach_irq
 ****************************************************************************/

void esp32s3_usbserial_debug_reattach_irq(void)
{
  esp32s3_usbserial_debug_reattach_irq_cpu(0);
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_reattach_irq_cpu
 ****************************************************************************/

void esp32s3_usbserial_debug_reattach_irq_cpu(int cpu)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;

#ifndef CONFIG_SMP
  cpu = 0;
#else
  if (cpu < 0 || cpu >= CONFIG_SMP_NCPUS)
    {
      cpu = 0;
    }
#endif

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  if (priv->cpuint != -ENOMEM)
    {
      esp32s3_teardown_irq(priv->cpu, priv->periph, priv->cpuint);
      priv->cpuint = -ENOMEM;
    }

  priv->cpu = cpu;
  priv->cpuint = esp32s3_setup_irq(priv->cpu, priv->periph,
                                   ESP32S3_INT_PRIO_DEF,
                                   ESP32S3_CPUINT_LEVEL);
  if (priv->cpuint < 0)
    {
      return;
    }

  if (irq_attach(priv->irq, esp32s3_interrupt, &g_uart_usbserial) == OK)
    {
      putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
                  USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
      up_enable_irq(priv->irq);
    }
}

/****************************************************************************
 * Name: esp32s3_usbserial_recover_rx
 ****************************************************************************/

void esp32s3_usbserial_recover_rx(void)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;

  putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
           USB_SERIAL_JTAG_INT_CLR_REG);
  modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
              USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
  up_enable_irq(priv->irq);
  esp32s3_usbserial_start_poll("rx-recover");
}

/****************************************************************************
 * Name: esp32s3_usbserial_reopen
 ****************************************************************************/

void esp32s3_usbserial_reopen(void)
{
  uart_dev_t *dev = &g_uart_usbserial;
  irqstate_t flags;

  flags = enter_critical_section();
  uart_disablerxint(dev);
  uart_detach(dev);
  dev->priv = &g_usbserial_priv;
  g_usbserial_priv.cpuint = -ENOMEM;
  uart_attach(dev);
  uart_enablerxint(dev);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: esp32s3_usbserial_recover_level
 ****************************************************************************/

void esp32s3_usbserial_recover_level(int level)
{
  uint32_t conf0;

  if (level >= 1)
    {
      esp32s3_usbserial_recover_rx();
    }

  if (level >= 2)
    {
      esp32s3_usbserial_reopen();
    }

  if (level >= 3)
    {
      modifyreg32(USB_SERIAL_JTAG_MISC_CONF_REG, 0, USB_SERIAL_JTAG_CLK_EN);
      modifyreg32(USB_SERIAL_JTAG_MEM_CONF_REG, USB_SERIAL_JTAG_USB_MEM_PD,
                  USB_SERIAL_JTAG_USB_MEM_CLK_EN);
      putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR |
               USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT_CLR |
               USB_SERIAL_JTAG_PID_ERR_INT_CLR |
               USB_SERIAL_JTAG_CRC5_ERR_INT_CLR |
               USB_SERIAL_JTAG_CRC16_ERR_INT_CLR |
               USB_SERIAL_JTAG_STUFF_ERR_INT_CLR |
               USB_SERIAL_JTAG_OUT_EP1_ZERO_PAYLOAD_INT_CLR |
               USB_SERIAL_JTAG_OUT_EP2_ZERO_PAYLOAD_INT_CLR,
               USB_SERIAL_JTAG_INT_CLR_REG);
      esp32s3_usbserial_reopen();
    }

  if (level >= 4)
    {
      conf0 = getreg32(USB_SERIAL_JTAG_CONF0_REG);
      conf0 |= USB_SERIAL_JTAG_USB_PAD_ENABLE |
               USB_SERIAL_JTAG_PAD_PULL_OVERRIDE |
               USB_SERIAL_JTAG_DP_PULLUP;
      conf0 &= ~(USB_SERIAL_JTAG_DM_PULLUP |
                 USB_SERIAL_JTAG_PULLUP_VALUE |
                 USB_SERIAL_JTAG_EXCHG_PINS_OVERRIDE |
                 USB_SERIAL_JTAG_EXCHG_PINS);
      putreg32(conf0, USB_SERIAL_JTAG_CONF0_REG);
      esp32s3_usbserial_reopen();
    }
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_restore_rx
 ****************************************************************************/

void esp32s3_usbserial_debug_restore_rx_nopoll(void)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;

  modifyreg32(USB_SERIAL_JTAG_MISC_CONF_REG, 0, USB_SERIAL_JTAG_CLK_EN);
  modifyreg32(USB_SERIAL_JTAG_MEM_CONF_REG, USB_SERIAL_JTAG_USB_MEM_PD,
              USB_SERIAL_JTAG_USB_MEM_CLK_EN);
  modifyreg32(USB_SERIAL_JTAG_JFIFO_ST_REG, 0,
              USB_SERIAL_JTAG_IN_FIFO_RESET |
              USB_SERIAL_JTAG_OUT_FIFO_RESET);
  usleep(1000);
  modifyreg32(USB_SERIAL_JTAG_JFIFO_ST_REG,
              USB_SERIAL_JTAG_IN_FIFO_RESET |
              USB_SERIAL_JTAG_OUT_FIFO_RESET, 0);

  putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
           USB_SERIAL_JTAG_INT_CLR_REG);
  up_enable_irq(priv->irq);
  modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
              USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_restore_rx
 ****************************************************************************/

void esp32s3_usbserial_debug_restore_rx(void)
{
  esp32s3_usbserial_debug_restore_rx_nopoll();
  esp32s3_usbserial_start_poll("restore_rx");
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_start_rxpoll
 ****************************************************************************/

void esp32s3_usbserial_debug_start_rxpoll(void)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;

  putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
           USB_SERIAL_JTAG_INT_CLR_REG);
  up_enable_irq(priv->irq);
  modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
              USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);

  esp32s3_usbserial_start_poll("rxpoll");
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_enable_rx_irqonly
 ****************************************************************************/

void esp32s3_usbserial_debug_enable_rx_irqonly(void)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;

  putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
           USB_SERIAL_JTAG_INT_CLR_REG);
  up_enable_irq(priv->irq);
  modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
              USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);
}

/****************************************************************************
 * Name: esp32s3_usbserial_raw_probe
 ****************************************************************************/

int esp32s3_usbserial_raw_probe(int seconds, uint32_t *last_ep1conf,
                                uint32_t *last_out1,
                                uint32_t *last_intst)
{
  uint32_t loops;
  uint32_t i;
  int seen = 0;

  if (seconds < 1)
    {
      seconds = 1;
    }

  loops = (uint32_t)seconds * 20;
  if (loops > 20)
    {
      loops = 20;
    }
  esp32s3_usbserial_force_pad("raw-probe");
  modifyreg32(USB_SERIAL_JTAG_INT_ENA_REG, 0,
              USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_ENA);

  printf("USB raw probe: begin nonblock seconds=%d, type keys now\n",
         seconds);
  fflush(stdout);

  for (i = 0; i < loops; i++)
    {
      uint32_t ep1conf = getreg32(USB_SERIAL_JTAG_EP1_CONF_REG);
      uint32_t out1 = getreg32(USB_SERIAL_JTAG_OUT_EP1_ST_REG);
      uint32_t count;
      volatile uint32_t spin;

      if (last_ep1conf != NULL)
        {
          *last_ep1conf = ep1conf;
        }

      if (last_out1 != NULL)
        {
          *last_out1 = out1;
        }

      if (last_intst != NULL)
        {
          *last_intst = getreg32(USB_SERIAL_JTAG_INT_ST_REG);
        }

      esp32s3_usbserial_trywrite('.');

      if ((ep1conf & USB_SERIAL_JTAG_SERIAL_OUT_EP_DATA_AVAIL) != 0)
        {
          int j;

          count = (out1 & USB_SERIAL_JTAG_OUT_EP1_REC_DATA_CNT_M) >>
                  USB_SERIAL_JTAG_OUT_EP1_REC_DATA_CNT_S;
          if (count == 0)
            {
              count = 64;
            }

          esp32s3_usbserial_trywrite('R');
          for (j = 0; j < count; j++)
            {
              int ch = getreg32(USB_SERIAL_JTAG_EP1_REG) &
                       USB_SERIAL_JTAG_RDWR_BYTE;

              if (ch >= 0x20 && ch <= 0x7e)
                {
                  esp32s3_usbserial_trywrite(ch);
                }
            }

          esp32s3_usbserial_trywrite('\n');
          putreg32(USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT_INT_CLR,
                   USB_SERIAL_JTAG_INT_CLR_REG);
          seen++;
        }

      for (spin = 0; spin < 30000; spin++)
        {
          __asm__ __volatile__("nop");
        }
    }

  esp32s3_usbserial_trywrite('\n');
  esp32s3_usbserial_trywrite('E');
  esp32s3_usbserial_trywrite('0' + (seen % 10));
  esp32s3_usbserial_trywrite('\n');
  return seen;
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_state
 ****************************************************************************/

void esp32s3_usbserial_debug_state(const char *tag)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;

  printf("usbserial: state %s cpu=%d open=%u console=%u pid=%d "
         "rxavail=%d txready=%d recv=%d/%d/%d xmit=%d/%d/%d "
         "cpuint=%d irq=%d intena=%08" PRIx32 " intst=%08" PRIx32
         " ep1conf=%08" PRIx32 " jfifo=%08" PRIx32 "\n",
         tag, sched_getcpu(), g_uart_usbserial.open_count,
         g_uart_usbserial.isconsole ? 1 : 0,
#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGTSTP) || \
    defined(CONFIG_TTY_FORCE_PANIC) || defined(CONFIG_TTY_LAUNCH)
         g_uart_usbserial.pid,
#else
         -2,
#endif
         esp32s3_rxavailable(&g_uart_usbserial) ? 1 : 0,
         esp32s3_txready(&g_uart_usbserial) ? 1 : 0,
         g_uart_usbserial.recv.head, g_uart_usbserial.recv.tail,
         g_uart_usbserial.recv.size,
         g_uart_usbserial.xmit.head, g_uart_usbserial.xmit.tail,
         g_uart_usbserial.xmit.size,
         priv->cpuint, priv->irq,
         getreg32(USB_SERIAL_JTAG_INT_ENA_REG),
         getreg32(USB_SERIAL_JTAG_INT_ST_REG),
         getreg32(USB_SERIAL_JTAG_EP1_CONF_REG),
         getreg32(USB_SERIAL_JTAG_JFIFO_ST_REG));
  fflush(stdout);
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_poll_state
 ****************************************************************************/

void esp32s3_usbserial_debug_poll_state(const char *tag)
{
  printf("usbserial: pollstate %s started=%" PRIu32 " heartbeat=%" PRIu32
         " created=%d\n",
         tag, g_usbserial_poll_started_seen,
         g_usbserial_poll_heartbeat_seen,
         g_usbserial_poll_started ? 1 : 0);
  fflush(stdout);
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_dump
 ****************************************************************************/

void esp32s3_usbserial_debug_dump(const char *tag)
{
  struct esp32s3_priv_s *priv = g_uart_usbserial.priv;
  uint32_t map0 = getreg32(INTERRUPT_CORE0_USB_DEVICE_INT_MAP_REG);
#ifdef CONFIG_SMP
  uint32_t map1 = getreg32(INTERRUPT_CORE1_USB_DEVICE_INT_MAP_REG);
#else
  uint32_t map1 = 0xffffffff;
#endif

  printf("usbserial: %s runcpu=%d ps=%08" PRIx32
         " cpu=%d cpuint=%d irq=%d periph=%d map0=%" PRIu32
         " map1=%" PRIu32 " conf0=%08" PRIx32
         " ep1=%08" PRIx32 " ep1conf=%08" PRIx32 " intena=%08" PRIx32
         " intst=%08" PRIx32 " jfifo=%08" PRIx32 " mem=%08" PRIx32
         " misc=%08" PRIx32 "\n",
         tag, sched_getcpu(), xtensa_getps(), priv->cpu, priv->cpuint,
         priv->irq, priv->periph, map0, map1,
         getreg32(USB_SERIAL_JTAG_CONF0_REG),
         getreg32(USB_SERIAL_JTAG_EP1_REG),
         getreg32(USB_SERIAL_JTAG_EP1_CONF_REG),
         getreg32(USB_SERIAL_JTAG_INT_ENA_REG),
         getreg32(USB_SERIAL_JTAG_INT_ST_REG),
         getreg32(USB_SERIAL_JTAG_JFIFO_ST_REG),
         getreg32(USB_SERIAL_JTAG_MEM_CONF_REG),
         getreg32(USB_SERIAL_JTAG_MISC_CONF_REG));
  fflush(stdout);
}

/****************************************************************************
 * Name: esp32s3_usbserial_debug_wait_txready
 ****************************************************************************/

void esp32s3_usbserial_debug_wait_txready(void)
{
  int loops;

  for (loops = 0; loops < 2000; loops++)
    {
      if (esp32s3_txready(&g_uart_usbserial))
        {
          break;
        }

      usleep(1000);
    }

  UNUSED(loops);
}
