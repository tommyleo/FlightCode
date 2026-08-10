#include "mpu6000.h"

#include "board.h"

#define REG_CONFIG 0x1AU
#define REG_SMPLRT_DIV 0x19U
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_ACCEL_XOUT_H 0x3BU
#define REG_SIGNAL_PATH_RESET 0x68U
#define REG_PWR_MGMT_1 0x6BU
#define REG_USER_CTRL 0x6AU
#define REG_WHO_AM_I 0x75U

static float gyro_lsb_per_dps = 131.0f;
static float accel_lsb_per_g = 16384.0f;
static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, data, 2, 10);
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

    /*
     * Keep command and payload in one full-duplex transaction.  Using
     * HAL_SPI_Receive() in 2-line master mode aliases its TX and RX buffers;
     * that corrupted alternating 16-bit sensor words on some targets.
     */
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 1U, 10U);
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
    if (status == HAL_OK) {
        for (uint16_t i = 0U; i < length; ++i) {
            data[i] = rx[i + 1U];
        }
    }
    return status == HAL_OK;
}

static int16_t be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static bool set_spi_prescaler(uint32_t prescaler)
{
    /*
     * MPU6000 register configuration is limited to 1 MHz.  Sensor burst
     * reads may subsequently use the faster SPI clock.
     */
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

bool mpu6000_init(void)
{
    /* Keep reset and configuration transfers below 1 MHz. */
    if (!set_spi_prescaler(SPI_BAUDRATEPRESCALER_128)) {
        return false;
    }
    HAL_Delay(100);
    uint8_t who = 0;
    if (!read_regs(REG_WHO_AM_I, &who, 1) ||
        (who != 0x68U && who != 0x70U)) {
        return false;
    }

    if (!write_reg(REG_PWR_MGMT_1, 0x80U)) return false;
    HAL_Delay(100);
    if (!write_reg(REG_SIGNAL_PATH_RESET, 0x07U)) return false;
    HAL_Delay(100);
    if (!write_reg(REG_PWR_MGMT_1, 0x01U) ||
        !write_reg(REG_USER_CTRL, 0x10U) ||
        /* DLPF_CFG=0 enables the MPU6000 8 kHz gyro output rate. */
        !write_reg(REG_CONFIG, 0x00U) ||
        !write_reg(REG_SMPLRT_DIV, 0x00U) ||
        !write_reg(REG_GYRO_CONFIG, 0x18U) ||
        !write_reg(REG_ACCEL_CONFIG, 0x10U)) {
        return false;
    }

    HAL_Delay(10U);
    uint8_t gyro_config = 0U;
    uint8_t accel_config = 0U;
    /*
     * Read back the ranges, but do not reject a working IMU merely because a
     * clone/revision reports different non-range bits.  The effective scales
     * below are derived from the values actually returned by the device.
     */
    if (!read_regs(REG_GYRO_CONFIG, &gyro_config, 1U) ||
        !read_regs(REG_ACCEL_CONFIG, &accel_config, 1U)) {
        return false;
    }
    static const float gyro_sensitivity[4] =
        {131.0f, 65.5f, 32.8f, 16.4f};
    static const float accel_sensitivity[4] =
        {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    gyro_lsb_per_dps =
        gyro_sensitivity[(gyro_config >> 3U) & 0x03U];
    accel_lsb_per_g =
        accel_sensitivity[(accel_config >> 3U) & 0x03U];

    /* Use a conservative live-data SPI clock on every supported target. */
    if (!set_spi_prescaler(SPI_BAUDRATEPRESCALER_16)) {
        return false;
    }
    HAL_Delay(1U);
    return true;
}

bool mpu6000_read(imu_sample_t *sample)
{
    uint8_t data[14];
    if (!read_regs(REG_ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    sample->accel_x_g = (float)be16(&data[0]) / accel_lsb_per_g;
    sample->accel_y_g = (float)be16(&data[2]) / accel_lsb_per_g;
    sample->accel_z_g = (float)be16(&data[4]) / accel_lsb_per_g;
    sample->gyro_x_dps = (float)be16(&data[8]) / gyro_lsb_per_dps;
    sample->gyro_y_dps = (float)be16(&data[10]) / gyro_lsb_per_dps;
    sample->gyro_z_dps = (float)be16(&data[12]) / gyro_lsb_per_dps;

    return true;
}
