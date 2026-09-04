#include <nuttx/config.h>

#include <sys/param.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_i2s.h>
#include <nuttx/audio/es7210.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/signal.h>

#ifdef CONFIG_AUDIO_ES7210

struct es7210_regval_s
{
  uint8_t reg;
  uint8_t val;
};

struct es7210_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint32_t frequency;
  uint8_t address;
};

#define ES7210_MIC_GAIN         0x1d
#define ES7210_MIC_OFF_GAIN     0x00
#define ES7210_ADC_GAIN         0x0a
#define ES7210_ADC_OFF_GAIN     0x08
#define ES7210_ADC12_UNMUTE     0x00
#define ES7210_ADC34_MUTE       0x03
#define ES7210_MODE_SLAVE_2CH   0x10
#define ES7210_MIC12_POWER_ON   0x00
#define ES7210_MIC34_POWER_DOWN 0xff

static int es7210_writereg(FAR struct es7210_dev_s *priv, uint8_t reg,
                           uint8_t val)
{
  struct i2c_config_s config;
  uint8_t data[2];

  config.frequency = priv->frequency;
  config.address   = priv->address;
  config.addrlen   = 7;

  data[0] = reg;
  data[1] = val;

  return i2c_write(priv->i2c, &config, data, sizeof(data));
}

static int es7210_write_table(FAR struct es7210_dev_s *priv,
                              FAR const struct es7210_regval_s *table,
                              size_t count)
{
  size_t i;
  int ret;

  for (i = 0; i < count; i++)
    {
      ret = es7210_writereg(priv, table[i].reg, table[i].val);
      if (ret < 0)
        {
          auderr("ERROR: ES7210 write %02x <- %02x failed: %d\n",
                 table[i].reg, table[i].val, ret);
          return ret;
        }
    }

  return OK;
}

static int es7210_configure(FAR struct es7210_dev_s *priv)
{
  static const struct es7210_regval_s init[] =
    {
      {0x00, 0x41},
      {0x01, 0x3f},
      {0x09, 0x30},
      {0x0a, 0x30},
      {0x23, 0x2a},
      {0x22, 0x0a},
      {0x20, 0x0a},
      {0x21, 0x2a},
      {0x08, ES7210_MODE_SLAVE_2CH},
      {0x40, 0x43},
      {0x41, 0x70},
      {0x42, 0x70},
      {0x07, 0x20},
      {0x02, 0xc1},
      {0x11, 0x60},
      {0x12, 0x00},
      {0x14, ES7210_ADC34_MUTE},
      {0x15, ES7210_ADC12_UNMUTE},
      {0x01, 0x34},
      {0x06, 0x00},
      {0x47, ES7210_ADC_GAIN},
      {0x48, ES7210_ADC_GAIN},
      {0x49, ES7210_ADC_OFF_GAIN},
      {0x4a, ES7210_ADC_OFF_GAIN},
      {0x43, ES7210_MIC_GAIN},
      {0x44, ES7210_MIC_GAIN},
      {0x45, ES7210_MIC_OFF_GAIN},
      {0x46, ES7210_MIC_OFF_GAIN},
      {0x4b, ES7210_MIC12_POWER_ON},
      {0x4c, ES7210_MIC34_POWER_DOWN},
      {0x40, 0x43},
      {0x00, 0x71},
      {0x00, 0x41},
    };

  int ret;

  ret = es7210_writereg(priv, 0x00, 0xff);
  if (ret < 0)
    {
      auderr("ERROR: ES7210 reset write failed: %d\n", ret);
      return ret;
    }

  /* The codec needs time to complete the software reset before the next
   * register access, otherwise the next configuration writes can NACK on
   * cold boot.
   */

  nxsig_usleep(10000);

  ret = es7210_write_table(priv, init, nitems(init));
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(1000);
  syslog(LOG_INFO, "ES7210 capture ADC initialized at 0x%02x\n",
         priv->address);
  return OK;
}

FAR struct audio_lowerhalf_s *
  es7210_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es7210_lower_s *lower)
{
  struct es7210_dev_s priv;
  int ret;

  DEBUGASSERT(i2c != NULL && i2s != NULL && lower != NULL);

  priv.i2c       = i2c;
  priv.frequency = lower->frequency;
  priv.address   = lower->address;

  ret = es7210_configure(&priv);
  if (ret < 0)
    {
      auderr("ERROR: ES7210 initialization failed at 0x%02x: %d\n",
             priv.address, ret);
      return NULL;
    }

  return audio_i2s_initialize(i2s, false);
}

#endif /* CONFIG_AUDIO_ES7210 */
