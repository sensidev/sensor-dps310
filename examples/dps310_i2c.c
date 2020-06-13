#include "dps310_i2c.h"

// Atmel includes
#include "delay.h"

// App includes
#include "app_i2c.h"

// Sensor driver includes
#include "dps310_errors.h"


void dps310_i2c_init(void) {
}

void dps310_i2c_release(void) {
}

int8_t dps310_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint16_t count) {
    int8_t ret;
    uint8_t buff[1];

    buff[0] = reg;

    ret = app_i2c_write_wait(address, buff, 1);

    if (ret != STATUS_OK) {
        return DPS310_I2C_FAIL_ERROR;
    }

    delay_ms(10);

    ret = app_i2c_read_wait(address, data, count);

    if (ret != STATUS_OK) {
        return DPS310_I2C_FAIL_ERROR;
    }

    return DPS310_OK;
}

int8_t dps310_i2c_write(uint8_t address, uint8_t reg, const uint8_t *data, uint16_t count) {
    int8_t ret;
    uint16_t count_with_reg = count + 1;
    uint8_t buff[DPS310_I2C_MAX_BUFF_SIZE];

    buff[0] = reg;

    for (uint8_t i = 1; i < count_with_reg; i++) {
        buff[i] = data[i - 1];
    }

    ret = app_i2c_write_wait(address, buff, count_with_reg);

    if (ret != STATUS_OK) {
        return DPS310_I2C_FAIL_ERROR;
    }

    return DPS310_OK;
}

void dps310_i2c_delay_ms(uint32_t delay) {
    delay_ms(delay);
}