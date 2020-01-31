#ifndef SENSOR_DPS310_I2C_H
#define SENSOR_DPS310_I2C_H

#include "stdint.h"

#define DPS310_I2C_MAX_BUFF_SIZE 10

void dps310_i2c_init(void);

int8_t dps310_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count);

int8_t dps310_i2c_write(uint8_t address, uint8_t reg, const uint8_t *data, uint16_t count);

void dps310_i2c_wakeup();

void dps310_i2c_sleep();

void dps310_i2c_delay_ms(uint32_t delay);

#endif