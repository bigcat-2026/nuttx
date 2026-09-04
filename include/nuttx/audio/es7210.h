#ifndef __INCLUDE_NUTTX_AUDIO_ES7210_H
#define __INCLUDE_NUTTX_AUDIO_ES7210_H

#include <nuttx/config.h>

#ifdef CONFIG_AUDIO_ES7210

#include <stdint.h>

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO is required for audio subsystem support
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S is required by the ES7210 driver
#endif

#ifndef CONFIG_I2C
#  error CONFIG_I2C is required by the ES7210 driver
#endif

#ifndef CONFIG_AUDIO_I2S
#  error CONFIG_AUDIO_I2S is required by the ES7210 driver
#endif

struct es7210_lower_s
{
  uint32_t frequency;
  uint8_t  address;
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct i2c_master_s;
struct i2s_dev_s;
struct audio_lowerhalf_s;

FAR struct audio_lowerhalf_s *
  es7210_initialize(FAR struct i2c_master_s *i2c,
                    FAR struct i2s_dev_s *i2s,
                    FAR const struct es7210_lower_s *lower);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_AUDIO_ES7210 */

#endif /* __INCLUDE_NUTTX_AUDIO_ES7210_H */
