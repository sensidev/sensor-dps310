#include "dps310.h"
#include "dps310_i2c.h"
#include "dps310_errors.h"
#include "stdbool.h"

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} calibration_coefs_t;

static calibration_coefs_t g_coefs;
static uint8_t g_pressure_rate = DPS310_CFG_RATE_1_MEAS;
static uint8_t g_temperature_rate = DPS310_CFG_RATE_1_MEAS;
static float g_last_temp_raw_sc;

static int16_t read_coefs();

static int32_t get_two_complement_of(uint32_t value, uint8_t length);

static int16_t write_byte_to_reg(uint8_t reg_addr, uint8_t data);

static int16_t wait_for_reg_value(uint8_t reg_addr, uint8_t reg_value, uint8_t mask);

static int16_t get_scale_factor_for(uint8_t rate, uint32_t *p_factor);

static int16_t read_temperature(float *p_temperature);

static int16_t read_pressure(float *p_pressure);

static int16_t correct_temperature();

static int16_t product_id_check();

static int16_t get_temperature_sensor(uint8_t *p_sensor);

int16_t dps310_probe() {
    int16_t ret;

    ret = product_id_check();
    if (ret != DPS310_OK) return ret;

    dps310_configure_temperature(
            DPS310_CFG_RATE_1_MEAS |
            DPS310_TMP_CFG_TMP_PRC_SINGLE);

    dps310_configure_pressure(
            DPS310_CFG_RATE_1_MEAS |
            DPS310_PRS_CFG_PM_PRC_2_TIMES);

    ret = read_coefs();
    if (ret != DPS310_OK) return ret;

    ret = dps310_sleep();
    if (ret != DPS310_OK) return ret;

    float trash;
    read_temperature(&trash);

    ret = dps310_sleep();
    if (ret != DPS310_OK) return ret;

    ret = correct_temperature();
    if (ret != DPS310_OK) return ret;

    return DPS310_OK;
}

int16_t dps310_wakeup(dps310_mode_t mode) {
    return write_byte_to_reg(DPS310_MEAS_CFG_REG, mode);
}

int16_t dps310_sleep() {
    return write_byte_to_reg(DPS310_MEAS_CFG_REG, DPS310_MEAS_CFG_MEAS_CTRL_IDLE);
}

int16_t dps310_reset() {
    return write_byte_to_reg(DPS310_RESET_REG, DPS310_RESET_SOFT_RST_VALUE);
}

int16_t read_coefs() {
    int16_t ret;
    uint8_t buff[18];

    ret = wait_for_reg_value(
            DPS310_MEAS_CFG_REG,
            DPS310_MEAS_CFG_COEF_RDY_AVAILABLE,
            DPS310_MEAS_CFG_COEF_RDY_AVAILABLE);

    ret = dps310_i2c_read(I2C_DPS310_ADDRESS, DPS310_COEF_REG, buff, 18);
    if (ret != DPS310_OK) return ret;

    g_coefs.c0 = get_two_complement_of(
            ((uint16_t) buff[0] << 4u) | (((uint16_t) buff[1] >> 4u) & 0x0Fu),
            12);

    g_coefs.c1 = get_two_complement_of(
            ((((uint16_t) buff[1] & 0x0Fu) << 8u) | (uint16_t) buff[2]),
            12);

    g_coefs.c00 = get_two_complement_of(
            ((uint32_t) buff[3] << 12u) | ((uint32_t) buff[4] << 4u) | (((uint32_t) buff[5] >> 4u) & 0x0Fu),
            20);

    g_coefs.c10 = get_two_complement_of(
            (((uint32_t) buff[5] & 0x0Fu) << 16u) | ((uint32_t) buff[6] << 8u) | (uint32_t) buff[7],
            20);

    g_coefs.c01 = get_two_complement_of(
            ((uint16_t) buff[8] << 8u) | (uint16_t) buff[9],
            16);

    g_coefs.c11 = get_two_complement_of(
            ((uint16_t) buff[10] << 8u) | (uint16_t) buff[11],
            16);

    g_coefs.c20 = get_two_complement_of(
            ((uint16_t) buff[12] << 8u) | (uint16_t) buff[13],
            16);

    g_coefs.c21 = get_two_complement_of(
            ((uint16_t) buff[14] << 8u) | (uint16_t) buff[15],
            16);

    g_coefs.c30 = get_two_complement_of(
            ((uint16_t) buff[16] << 8u) | (uint16_t) buff[17],
            16);

    return DPS310_OK;
}

int16_t dps310_configure_temperature(uint8_t data) {
    int16_t ret;
    uint8_t temperature_sensor = DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL;

    ret = get_temperature_sensor(&temperature_sensor);
    if (ret != DPS310_OK) return ret;

    g_temperature_rate = DPS310_TMP_CFG_TMP_RATE_MASK & data;
    data |= temperature_sensor;

    return write_byte_to_reg(DPS310_TMP_CFG_REG, data);
}

int16_t dps310_configure_pressure(uint8_t data) {
    g_pressure_rate = DPS310_PRS_CFG_PM_RATE_MASK & data;
    return write_byte_to_reg(DPS310_PRS_CFG_REG, data);
}

int16_t write_byte_to_reg(uint8_t reg_addr, uint8_t data) {
    int16_t ret;
    uint8_t buff[1];

    buff[0] = data;
    ret = dps310_i2c_write(I2C_DPS310_ADDRESS, reg_addr, buff, 1);
    if (ret != DPS310_OK) return ret;

    return DPS310_OK;
}

int16_t dps310_read(float *p_temperature, float *p_pressure) {
    int16_t ret;

    ret = read_temperature(p_temperature);
    if (ret != DPS310_OK) return ret;

    dps310_i2c_delay_ms(50);

    ret = read_pressure(p_pressure);
    if (ret != DPS310_OK) return ret;

    return DPS310_OK;
}

static int16_t read_temperature(float *p_temperature) {
    int16_t ret;
    uint8_t buff[3];

    ret = write_byte_to_reg(DPS310_MEAS_CFG_REG, DPS310_MEAS_CFG_MEAS_CTRL_TMP);
    if (ret != DPS310_OK) return ret;

    ret = wait_for_reg_value(
            DPS310_MEAS_CFG_REG,
            DPS310_MEAS_CFG_SENSOR_RDY_COMPLETE | DPS310_MEAS_CFG_TMP_RDY | DPS310_MEAS_CFG_MEAS_CTRL_IDLE,
            DPS310_MEAS_CFG_SENSOR_RDY_COMPLETE | DPS310_MEAS_CFG_TMP_RDY | DPS310_MEAS_CFG_MEAS_CTRL_MASK);
    if (ret != DPS310_OK) return ret;

    ret = dps310_i2c_read(I2C_DPS310_ADDRESS, DPS310_TMP_B2_REG, buff, 3);
    if (ret != DPS310_OK) return ret;

    int32_t temp_raw = get_two_complement_of(
            ((uint32_t) buff[0] << 16u) | ((uint32_t) buff[1] << 8u) | (uint32_t) buff[2],
            24);

    uint32_t factor;
    ret = get_scale_factor_for(g_temperature_rate, &factor);
    if (ret != DPS310_OK) return ret;

    g_last_temp_raw_sc = (float) temp_raw / factor;
    *p_temperature = (float) g_coefs.c0 * 0.5f + (float) g_coefs.c1 * g_last_temp_raw_sc;

    return DPS310_OK;
}

int16_t read_pressure(float *p_pressure) {
    int16_t ret;
    uint8_t buff[3];

    ret = write_byte_to_reg(DPS310_MEAS_CFG_REG, DPS310_MEAS_CFG_MEAS_CTRL_PRS);
    if (ret != DPS310_OK) return ret;

    ret = wait_for_reg_value(
            DPS310_MEAS_CFG_REG,
            DPS310_MEAS_CFG_SENSOR_RDY_COMPLETE | DPS310_MEAS_CFG_PRS_RDY | DPS310_MEAS_CFG_MEAS_CTRL_IDLE,
            DPS310_MEAS_CFG_SENSOR_RDY_COMPLETE | DPS310_MEAS_CFG_PRS_RDY | DPS310_MEAS_CFG_MEAS_CTRL_MASK);
    if (ret != DPS310_OK) return ret;

    ret = dps310_i2c_read(I2C_DPS310_ADDRESS, DPS310_PSR_B2_REG, buff, 3);
    if (ret != DPS310_OK) return ret;

    int32_t pressure_raw = get_two_complement_of(
            ((uint32_t) buff[0] << 16u) | ((uint32_t) buff[1] << 8u) | (uint32_t) buff[2],
            24);

    uint32_t factor;
    ret = get_scale_factor_for(g_pressure_rate, &factor);
    if (ret != DPS310_OK) return ret;

    float pressure_raw_sc = (float) pressure_raw / factor;

    *p_pressure = g_coefs.c00 +
                  pressure_raw_sc * (g_coefs.c10 + pressure_raw_sc * (g_coefs.c20 + pressure_raw_sc * g_coefs.c30)) +
                  g_last_temp_raw_sc * (g_coefs.c01 + pressure_raw_sc * (g_coefs.c11 + pressure_raw_sc * g_coefs.c21));

    return DPS310_OK;
}

int16_t wait_for_reg_value(uint8_t reg_addr, uint8_t reg_value, uint8_t mask) {
    int16_t ret;
    uint8_t buff[1];
    uint16_t attempts = 0;

    while (attempts < DPS310_READ_WAIT_FOR_REG_ATTEMPTS) {
        attempts++;

        ret = dps310_i2c_read(I2C_DPS310_ADDRESS, reg_addr, buff, 1);
        if (ret != DPS310_OK) return ret;

        bool b_is_expected_value = ((buff[0] & mask) == reg_value);
        if (b_is_expected_value) return DPS310_OK;

        dps310_i2c_delay_ms(10);
    }

    if (attempts == DPS310_READ_WAIT_FOR_REG_ATTEMPTS) {
        return DPS310_WAIT_TIMEOUT_ERROR;
    }

    return DPS310_OK;
}

int16_t get_scale_factor_for(uint8_t rate, uint32_t *p_factor) {
    uint32_t ret = DPS310_OK;

    switch (rate) {
        case DPS310_CFG_RATE_1_MEAS:
            *p_factor = 524288;
            break;
        case DPS310_CFG_RATE_2_MEAS:
            *p_factor = 1572864;
            break;
        case DPS310_CFG_RATE_4_MEAS:
            *p_factor = 3670016;
            break;
        case DPS310_CFG_RATE_8_MEAS:
            *p_factor = 7864320;
            break;
        case DPS310_CFG_RATE_16_MEAS:
            *p_factor = 253952;
            break;
        case DPS310_CFG_RATE_32_MEAS:
            *p_factor = 516096;
            break;
        case DPS310_CFG_RATE_64_MEAS:
            *p_factor = 1040384;
            break;
        case DPS310_CFG_RATE_128_MEAS:
            *p_factor = 2088960;
            break;
        default:
            ret = DPS310_UNKNOWN_RATE_ERROR;
    }

    return ret;
}

int32_t get_two_complement_of(uint32_t value, uint8_t length) {
    int32_t ret = value;
    bool b_is_negative = value & (1u << (length - 1u));

    if (b_is_negative) {
        ret -= ((uint32_t) 1 << length);
    }

    return ret;
}

int16_t correct_temperature() {
    int16_t ret;

    ret = write_byte_to_reg(0x0E, 0xA5);
    if (ret != DPS310_OK) return ret;

    ret = write_byte_to_reg(0x0F, 0x96);
    if (ret != DPS310_OK) return ret;

    ret = write_byte_to_reg(0x62, 0x02);
    if (ret != DPS310_OK) return ret;

    ret = write_byte_to_reg(0x0E, 0x00);
    if (ret != DPS310_OK) return ret;

    ret = write_byte_to_reg(0x0F, 0x00);
    if (ret != DPS310_OK) return ret;

    return DPS310_OK;
}

int16_t product_id_check() {
    int16_t ret;
    uint8_t buff[1];

    ret = dps310_i2c_read(I2C_DPS310_ADDRESS, DPS310_PRODUCT_ID_REG, buff, 1);
    if (ret != DPS310_OK) return ret;

    bool b_is_product_id_valid = buff[0] == DPS310_PRODUCT_ID_VALUE;
    if (!b_is_product_id_valid) return DPS310_PRODUCT_ID_ERROR;

    return DPS310_OK;
}

int16_t get_temperature_sensor(uint8_t *p_sensor) {
    uint16_t ret;
    uint8_t buff[1];

    ret = dps310_i2c_read(I2C_DPS310_ADDRESS, DPS310_TMP_COEF_SRCE, buff, 1);
    if (ret != DPS310_OK) return ret;

    uint8_t value = buff[0] & DPS310_TMP_COEF_SRCE_MASK;

    if (value) {
        *p_sensor = DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL;
    } else {
        *p_sensor = DPS310_TMP_CFG_REG_TMP_EXT_INTERNAL;
    }

    return DPS310_OK;
}