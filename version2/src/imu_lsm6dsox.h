#ifndef IMU_LSM6DSOX_H
#define IMU_LSM6DSOX_H

#include <stdint.h>

#define IMU_I2C_BUS 1

/* Init I2C + sensor; sets *addr_out to 0x6A or 0x6B. Returns 0 on success. */
int imu_lsm6dsox_init(uint8_t *addr_out);

/* Read raw gyro + accel. Returns 0 on success. */
int imu_lsm6dsox_read(uint8_t addr,
                      int16_t *gx, int16_t *gy, int16_t *gz,
                      int16_t *ax, int16_t *ay, int16_t *az);

#endif
