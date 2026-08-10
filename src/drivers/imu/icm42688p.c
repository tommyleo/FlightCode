#include "icm42688p.h"

#include "board.h"

#define REG_DEVICE_CONFIG 0x11U
#define REG_TEMP_DATA1 0x1DU
#define REG_INTF_CONFIG1 0x4DU
#define REG_PWR_MGMT0 0x4EU
#define REG_GYRO_CONFIG0 0x4FU
#define REG_ACCEL_CONFIG0 0x50U
#define REG_GYRO_ACCEL_CONFIG0 0x52U
#define REG_BANK_SELECT 0x76U
#define REG_WHO_AM_I 0x75U

#define REG_GYRO_CONFIG_STATIC3 0x0CU
#define REG_GYRO_CONFIG_STATIC4 0x0DU
#define REG_GYRO_CONFIG_STATIC5 0x0EU
#define REG_ACCEL_CONFIG_STATIC2 0x03U
#define REG_ACCEL_CONFIG_STATIC3 0x04U
#define REG_ACCEL_CONFIG_STATIC4 0x05U

#define WHO_AM_I_ICM42688P 0x47U
#define DEVICE_SOFT_RESET 0x01U
#define PWR_GYRO_ACCEL_LOW_NOISE 0x0FU
#define ODR_8KHZ_FS_MAX 0x03U
#define UI_FILTER_LOW_LATENCY 0xFFU
#define INTF_CONFIG1_AFSR_MASK 0xC0U
#define INTF_CONFIG1_AFSR_DISABLE 0x40U

#define GYRO_LSB_PER_DPS 16.4f
#define ACCEL_LSB_PER_G 2048.0f

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi1, data, sizeof(data), 10U);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
    return status == HAL_OK;
}

static bool read_regs(uint8_t reg, uint8_t *data, uint16_t length)
{
    if (length == 0U || length > 14U) {
        return false;
    }

    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};
    tx[0] = reg | 0x80U;
    for (uint16_t i = 1U; i <= length; ++i) {
        tx[i] = 0xFFU;
    }

    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 1U, 10U);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
    if (status != HAL_OK) {
        return false;
    }
    for (uint16_t i = 0U; i < length; ++i) {
        data[i] = rx[i + 1U];
    }
    return true;
}

static bool read_reg(uint8_t reg, uint8_t *value)
{
    return read_regs(reg, value, 1U);
}

static bool set_spi_prescaler(uint32_t prescaler)
{
    const uint32_t started = HAL_GetTick();
    while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET) {
        if ((HAL_GetTick() - started) > 10U) {
            return false;
        }
    }
    __HAL_SPI_DISABLE(&hspi1);
    MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR, prescaler);
    hspi1.Init.BaudRatePrescaler = prescaler;
    __HAL_SPI_ENABLE(&hspi1);
    return true;
}

static int16_t be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

bool icm42688p_init(void)
{
    /* Configure and identify the sensor below its 1 MHz startup limit. */
    if (!set_spi_prescaler(SPI_BAUDRATEPRESCALER_128)) {
        return false;
    }
    HAL_Delay(2U);
    if (!write_reg(REG_BANK_SELECT, 0U) ||
        !write_reg(REG_DEVICE_CONFIG, DEVICE_SOFT_RESET)) {
        return false;
    }
    HAL_Delay(2U);
    if (!write_reg(REG_BANK_SELECT, 0U) ||
        !write_reg(REG_PWR_MGMT0, 0U)) {
        return false;
    }

    uint8_t who = 0U;
    uint8_t attempts = 20U;
    do {
        HAL_Delay(1U);
        if (read_reg(REG_WHO_AM_I, &who) && who == WHO_AM_I_ICM42688P) {
            break;
        }
    } while (--attempts != 0U);
    if (who != WHO_AM_I_ICM42688P) {
        return false;
    }

    /* Betaflight's 258 Hz anti-alias configuration for the ICM-42688-P. */
    if (!write_reg(REG_BANK_SELECT, 1U) ||
        !write_reg(REG_GYRO_CONFIG_STATIC3, 6U) ||
        !write_reg(REG_GYRO_CONFIG_STATIC4, 36U) ||
        !write_reg(REG_GYRO_CONFIG_STATIC5, 0xA0U) ||
        !write_reg(REG_BANK_SELECT, 2U) ||
        !write_reg(REG_ACCEL_CONFIG_STATIC2, 12U) ||
        !write_reg(REG_ACCEL_CONFIG_STATIC3, 36U) ||
        !write_reg(REG_ACCEL_CONFIG_STATIC4, 0xA0U) ||
        !write_reg(REG_BANK_SELECT, 0U) ||
        !write_reg(REG_GYRO_ACCEL_CONFIG0, UI_FILTER_LOW_LATENCY)) {
        return false;
    }

    uint8_t interface_config = 0U;
    if (!read_reg(REG_INTF_CONFIG1, &interface_config)) {
        return false;
    }
    interface_config =
        (interface_config & ~INTF_CONFIG1_AFSR_MASK) |
        INTF_CONFIG1_AFSR_DISABLE;
    if (!write_reg(REG_INTF_CONFIG1, interface_config) ||
        !write_reg(REG_PWR_MGMT0, PWR_GYRO_ACCEL_LOW_NOISE)) {
        return false;
    }
    HAL_Delay(1U);
    if (!write_reg(REG_GYRO_CONFIG0, ODR_8KHZ_FS_MAX)) {
        return false;
    }
    HAL_Delay(15U);
    if (!write_reg(REG_ACCEL_CONFIG0, ODR_8KHZ_FS_MAX)) {
        return false;
    }
    HAL_Delay(15U);

    /* 168 MHz F405 configuration gives a safe 10.5 MHz SPI clock. */
    return set_spi_prescaler(SPI_BAUDRATEPRESCALER_8);
}

bool icm42688p_read(imu_sample_t *sample)
{
    uint8_t data[14];
    if (!read_regs(REG_TEMP_DATA1, data, sizeof(data))) {
        return false;
    }

    sample->accel_x_g = (float)be16(&data[2]) / ACCEL_LSB_PER_G;
    sample->accel_y_g = (float)be16(&data[4]) / ACCEL_LSB_PER_G;
    sample->accel_z_g = (float)be16(&data[6]) / ACCEL_LSB_PER_G;
    sample->gyro_x_dps = (float)be16(&data[8]) / GYRO_LSB_PER_DPS;
    sample->gyro_y_dps = (float)be16(&data[10]) / GYRO_LSB_PER_DPS;
    sample->gyro_z_dps = (float)be16(&data[12]) / GYRO_LSB_PER_DPS;
    return true;
}
