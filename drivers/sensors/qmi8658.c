/****************************************************************************
 * drivers/sensors/qmi8658.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/qmi8658.h>
#ifdef CONFIG_UORB
#  include <nuttx/sensors/sensor.h>
#  include <nuttx/wqueue.h>
#endif

#if defined(CONFIG_SENSORS_QMI8658)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QMI8658_WHO_AM_I        0x00
#define QMI8658_CTRL1           0x02
#define QMI8658_CTRL2           0x03
#define QMI8658_CTRL3           0x04
#define QMI8658_CTRL7           0x08
#define QMI8658_AX_L            0x35
#define QMI8658_DEVID           0x05

#define QMI8658_DEFAULT_INTERVAL_US  20000
#define QMI8658_MIN_INTERVAL_US      10000
#define QMI8658_MAX_INTERVAL_US      1000000
#define QMI8658_ACCEL_SCALE          (9.80665f / 4096.0f)
#define QMI8658_GYRO_SCALE           (0.01745329252f / 64.0f)

#ifndef CONFIG_QMI8658_I2C_ADDR
#  define CONFIG_QMI8658_I2C_ADDR 0x6a
#endif

#ifndef CONFIG_QMI8658_I2C_FREQUENCY
#  define CONFIG_QMI8658_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct qmi8658_axis_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct qmi8658_sample_s
{
  struct qmi8658_axis_s gyro;
  struct qmi8658_axis_s accel;
  uint32_t sensor_time;
};

struct qmi8658_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  uint32_t freq;
};

#ifdef CONFIG_UORB
struct qmi8658_sensor_s
{
  struct sensor_lowerhalf_s lower;
  struct qmi8658_dev_s dev;
  struct work_s work;
  uint32_t interval;
  bool enabled;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int qmi8658_open(FAR struct file *filep);
static int qmi8658_close(FAR struct file *filep);
static ssize_t qmi8658_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);

#ifdef CONFIG_UORB
static int qmi8658_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep, bool enable);
static int qmi8658_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                       FAR struct file *filep,
                                       FAR uint32_t *period_us);
static void qmi8658_sensor_worker(FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_qmi8658_fops =
{
  qmi8658_open,
  qmi8658_close,
  qmi8658_read,
  NULL,
  NULL,
  NULL,
};

#ifdef CONFIG_UORB
static const struct sensor_ops_s g_qmi8658_sensor_ops =
{
  .activate     = qmi8658_sensor_activate,
  .set_interval = qmi8658_sensor_set_interval,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int qmi8658_getregs(FAR struct qmi8658_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *buffer,
                           size_t buflen)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = buflen;

  return I2C_TRANSFER(priv->i2c, msg, 2);
}

static int qmi8658_getreg8(FAR struct qmi8658_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval)
{
  return qmi8658_getregs(priv, regaddr, regval, 1);
}

static int qmi8658_putreg8(FAR struct qmi8658_dev_s *priv,
                           uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];

  buffer[0] = regaddr;
  buffer[1] = regval;

  msg.frequency = priv->freq;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = sizeof(buffer);

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

static int qmi8658_configure(FAR struct qmi8658_dev_s *priv)
{
  int ret;

  ret = qmi8658_putreg8(priv, QMI8658_CTRL1, 0x60);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_putreg8(priv, QMI8658_CTRL2, 0x95);
  if (ret < 0)
    {
      return ret;
    }

  ret = qmi8658_putreg8(priv, QMI8658_CTRL3, 0xd5);
  if (ret < 0)
    {
      return ret;
    }

  return qmi8658_putreg8(priv, QMI8658_CTRL7, 0x03);
}

static int qmi8658_checkid(FAR struct qmi8658_dev_s *priv)
{
  uint8_t devid;
  int ret;

  ret = qmi8658_getreg8(priv, QMI8658_WHO_AM_I, &devid);
  if (ret < 0)
    {
      return ret;
    }

  if (devid != QMI8658_DEVID)
    {
      snerr("Wrong QMI8658 device ID: 0x%02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

static int qmi8658_read_sample(FAR struct qmi8658_dev_s *priv,
                               FAR struct qmi8658_sample_s *sample)
{
  uint8_t raw[12];
  int ret;

  ret = qmi8658_getregs(priv, QMI8658_AX_L, raw, sizeof(raw));
  if (ret < 0)
    {
      return ret;
    }

  memset(sample, 0, sizeof(*sample));

  sample->accel.x = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
  sample->accel.y = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
  sample->accel.z = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));
  sample->gyro.x  = (int16_t)((uint16_t)raw[6] | ((uint16_t)raw[7] << 8));
  sample->gyro.y  = (int16_t)((uint16_t)raw[8] | ((uint16_t)raw[9] << 8));
  sample->gyro.z  = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

  return OK;
}

#ifdef CONFIG_UORB
static void qmi8658_queue_worker(FAR struct qmi8658_sensor_s *sensor)
{
  clock_t delay;

  delay = USEC2TICK(sensor->interval);
  if (delay < 1)
    {
      delay = 1;
    }

  work_queue(HPWORK, &sensor->work, qmi8658_sensor_worker, sensor, delay);
}

static void qmi8658_sensor_worker(FAR void *arg)
{
  FAR struct qmi8658_sensor_s *sensor = arg;
  struct qmi8658_sample_s sample;
  int ret;

  if (!sensor->enabled)
    {
      return;
    }

  ret = qmi8658_read_sample(&sensor->dev, &sample);
  if (ret >= 0)
    {
      if (sensor->lower.type == SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED)
        {
          struct sensor_accel_uncal accel;

          memset(&accel, 0, sizeof(accel));
          accel.timestamp = sensor_get_timestamp();
          accel.x = sample.accel.x * QMI8658_ACCEL_SCALE;
          accel.y = sample.accel.y * QMI8658_ACCEL_SCALE;
          accel.z = sample.accel.z * QMI8658_ACCEL_SCALE;
          sensor->lower.push_event(sensor->lower.priv, &accel,
                                   sizeof(accel));
        }
      else
        {
          struct sensor_gyro_uncal gyro;

          memset(&gyro, 0, sizeof(gyro));
          gyro.timestamp = sensor_get_timestamp();
          gyro.x = sample.gyro.x * QMI8658_GYRO_SCALE;
          gyro.y = sample.gyro.y * QMI8658_GYRO_SCALE;
          gyro.z = sample.gyro.z * QMI8658_GYRO_SCALE;
          sensor->lower.push_event(sensor->lower.priv, &gyro,
                                   sizeof(gyro));
        }
    }

  if (sensor->enabled)
    {
      qmi8658_queue_worker(sensor);
    }
}

static int qmi8658_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep, bool enable)
{
  FAR struct qmi8658_sensor_s *sensor =
    (FAR struct qmi8658_sensor_s *)lower;
  int ret;

  if (enable)
    {
      ret = qmi8658_configure(&sensor->dev);
      if (ret < 0)
        {
          return ret;
        }

      if (!sensor->enabled)
        {
          sensor->enabled = true;
          qmi8658_queue_worker(sensor);
        }
    }
  else
    {
      sensor->enabled = false;
      work_cancel(HPWORK, &sensor->work);
    }

  return OK;
}

static int qmi8658_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                       FAR struct file *filep,
                                       FAR uint32_t *period_us)
{
  FAR struct qmi8658_sensor_s *sensor =
    (FAR struct qmi8658_sensor_s *)lower;

  if (period_us == NULL)
    {
      return -EINVAL;
    }

  if (*period_us < QMI8658_MIN_INTERVAL_US)
    {
      *period_us = QMI8658_MIN_INTERVAL_US;
    }
  else if (*period_us > QMI8658_MAX_INTERVAL_US)
    {
      *period_us = QMI8658_MAX_INTERVAL_US;
    }

  sensor->interval = *period_us;

  if (sensor->enabled)
    {
      work_cancel(HPWORK, &sensor->work);
      qmi8658_queue_worker(sensor);
    }

  return OK;
}
#endif

static int qmi8658_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct qmi8658_dev_s *priv = inode->i_private;

  return qmi8658_configure(priv);
}

static int qmi8658_close(FAR struct file *filep)
{
  return 0;
}

static ssize_t qmi8658_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct qmi8658_dev_s *priv = inode->i_private;
  FAR struct qmi8658_sample_s *sample;
  int ret;

  if (buflen < sizeof(struct qmi8658_sample_s))
    {
      snerr("Expected buffer size is %zu\n",
            sizeof(struct qmi8658_sample_s));
      return 0;
    }

  sample = (FAR struct qmi8658_sample_s *)buffer;

  ret = qmi8658_read_sample(priv, sample);
  if (ret < 0)
    {
      return ret;
    }

  return sizeof(struct qmi8658_sample_s);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int qmi8658_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
  FAR struct qmi8658_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct qmi8658_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = CONFIG_QMI8658_I2C_ADDR;
  priv->freq = CONFIG_QMI8658_I2C_FREQUENCY;

  ret = qmi8658_checkid(priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  ret = qmi8658_configure(priv);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  ret = register_driver(devpath, &g_qmi8658_fops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}

#ifdef CONFIG_UORB
static int qmi8658_register_sensor(int devno, FAR struct i2c_master_s *i2c,
                                   int type)
{
  FAR struct qmi8658_sensor_s *sensor;
  int ret;

  sensor = kmm_zalloc(sizeof(*sensor));
  if (sensor == NULL)
    {
      return -ENOMEM;
    }

  sensor->dev.i2c  = i2c;
  sensor->dev.addr = CONFIG_QMI8658_I2C_ADDR;
  sensor->dev.freq = CONFIG_QMI8658_I2C_FREQUENCY;
  sensor->interval = QMI8658_DEFAULT_INTERVAL_US;
  sensor->lower.ops = &g_qmi8658_sensor_ops;
  sensor->lower.type = type;
  sensor->lower.nbuffer = 2;

  ret = qmi8658_checkid(&sensor->dev);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ret;
    }

  ret = qmi8658_configure(&sensor->dev);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ret;
    }

  ret = sensor_register(&sensor->lower, devno);
  if (ret < 0)
    {
      kmm_free(sensor);
    }

  return ret;
}

int qmi8658_register_uorb(int devno, FAR struct i2c_master_s *i2c)
{
  int ret;

  ret = qmi8658_register_sensor(devno, i2c,
                                SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED);
  if (ret < 0)
    {
      return ret;
    }

  return qmi8658_register_sensor(devno, i2c,
                                 SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
}
#endif

#endif /* CONFIG_SENSORS_QMI8658 */
