#include <string.h>

#include "rpi_i2c.h"
#include "imu_lsm6dsox.h"

#define LSM6DSOX_ADDR0 0x6A
#define LSM6DSOX_ADDR1 0x6B

#define REG_WHO_AM_I   0x0F
#define REG_CTRL1_XL   0x10
#define REG_CTRL2_G    0x11
#define REG_CTRL3_C    0x12
#define REG_OUTX_L_G   0x22
#define REG_OUTX_L_A   0x28

#define WHO_AM_I_EXPECTED 0x6C

static int detect_addr(uint8_t *found_addr)
{
    uint8_t who = 0;

    if (smbus_read_byte_data(IMU_I2C_BUS, LSM6DSOX_ADDR0, REG_WHO_AM_I, &who) == I2C_SUCCESS) {
        if (who == WHO_AM_I_EXPECTED) {
            *found_addr = LSM6DSOX_ADDR0;
            return 0;
        }
    }
    if (smbus_read_byte_data(IMU_I2C_BUS, LSM6DSOX_ADDR1, REG_WHO_AM_I, &who) == I2C_SUCCESS) {
        if (who == WHO_AM_I_EXPECTED) {
            *found_addr = LSM6DSOX_ADDR1;
            return 0;
        }
    }
    return -1;
}

static int write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    return (smbus_write_byte_data(IMU_I2C_BUS, addr, reg, val) == I2C_SUCCESS) ? 0 : -1;
}

static int read_block(uint8_t addr, uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    return (smbus_read_block_data(IMU_I2C_BUS, addr, start_reg, buf, len) == I2C_SUCCESS) ? 0 : -1;
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int imu_lsm6dsox_init(uint8_t *addr_out)
{
    uint8_t addr = 0;

    if (addr_out == NULL) {
        return -1;
    }
    if (detect_addr(&addr) != 0) {
        return -1;
    }

    if (write_reg(addr, REG_CTRL3_C, 0x44) != 0) {
        return -1;
    }
    if (write_reg(addr, REG_CTRL1_XL, 0x40) != 0) {
        return -1;
    }
    if (write_reg(addr, REG_CTRL2_G, 0x40) != 0) {
        return -1;
    }

    *addr_out = addr;
    return 0;
}

int imu_lsm6dsox_read(uint8_t addr,
                      int16_t *gx, int16_t *gy, int16_t *gz,
                      int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t gbuf[6];
    uint8_t abuf[6];

    if (read_block(addr, REG_OUTX_L_G, gbuf, sizeof(gbuf)) != 0) {
        return -1;
    }
    if (read_block(addr, REG_OUTX_L_A, abuf, sizeof(abuf)) != 0) {
        return -1;
    }

    *gx = le16(&gbuf[0]);
    *gy = le16(&gbuf[2]);
    *gz = le16(&gbuf[4]);
    *ax = le16(&abuf[0]);
    *ay = le16(&abuf[2]);
    *az = le16(&abuf[4]);
    return 0;
}
