#ifndef __INCLUDE_NUTTX_SENSORS_QMI8658_H
#define __INCLUDE_NUTTX_SENSORS_QMI8658_H

#include <nuttx/config.h>
#include <nuttx/compiler.h>

struct i2c_master_s;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_SENSORS_QMI8658
int qmi8658_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c);

#  ifdef CONFIG_UORB
int qmi8658_register_uorb(int devno, FAR struct i2c_master_s *i2c);
#  endif
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_QMI8658_H */
