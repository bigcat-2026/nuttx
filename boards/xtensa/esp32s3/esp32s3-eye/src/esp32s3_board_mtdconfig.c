/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_mtdconfig.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdlib.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "esp32s3-eye.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_TESTING_MTD_CONFIG_MOUNTPT_NAME
#  define CONFIG_TESTING_MTD_CONFIG_MOUNTPT_NAME "/dev/mtdnvs_flash"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t *g_mtdconfig_test_ram;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_mtdconfig_test_initialize(void)
{
  struct mtd_dev_s *mtd;
  int ret;

  if (g_mtdconfig_test_ram == NULL)
    {
      g_mtdconfig_test_ram = malloc(64 * 1024);
      if (g_mtdconfig_test_ram == NULL)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to allocate RAM MTD for /dev/config\n");
          return -ENOMEM;
        }
    }

  mtd = rammtd_initialize(g_mtdconfig_test_ram,
                           64 * 1024);
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate RAM MTD for /dev/config\n");
      return -ENOMEM;
    }

  ret = register_mtddriver(CONFIG_TESTING_MTD_CONFIG_MOUNTPT_NAME, mtd, 0666,
                           NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD test device: %d\n",
             ret);
      return ret;
    }

  syslog(LOG_INFO, "MTD config test device ready: %s\n",
         CONFIG_TESTING_MTD_CONFIG_MOUNTPT_NAME);
  return OK;
}
