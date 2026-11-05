/****************************************************************************
 * include/nuttx/audio/es7210.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __INCLUDE_NUTTX_AUDIO_ES7210_H
#define __INCLUDE_NUTTX_AUDIO_ES7210_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#ifdef CONFIG_AUDIO_ES7210

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
