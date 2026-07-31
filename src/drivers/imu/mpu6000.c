#include "mpu6000.h"

#include <math.h>

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
static float alignment[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

void mpu6000_set_board_alignment(float roll_deg, float pitch_deg,
                                 float yaw_deg)
{
    const float to_rad = 0.01745329251994329577f;
    const float cr = cosf(roll_deg * to_rad);
    const float sr = sinf(roll_deg * to_rad);
    const float cp = cosf(pitch_deg * to_rad);
    const float sp = sinf(pitch_deg * to_rad);
    const float cy = cosf(yaw_deg * to_rad);
    const float sy = sinf(yaw_deg * to_rad);

    /* Rotate sensor-frame vectors into the quad body frame. */
    alignment[0][0] = cy * cp;
    alignment[0][1] = cy * sp * sr - sy * cr;
    alignment[0][2] = cy * sp * cr + sy * sr;
    alignment[1][0] = sy * cp;
    alignment[1][1] = sy * sp * sr + cy * cr;
    alignment[1][2] = sy * sp * cr - cy * sr;
    alignment[2][0] = -sp;
    alignment[2][1] = cp * sr;
    alignment[2][2] = cp * cr;
}

static void rotate(float *x, float *y, float *z)
{
    const float in_x = *x;
    const float in_y = *y;
    const float in_z = *z;
    *x = alignment[0][0] * in_x + alignment[0][1] * in_y +
         alignment[0][2] * in_z;
    *y = alignment[1][0] * in_x + alignment[1][1] * in_y +
         alignment[1][2] * in_z;
    *z = alignment[2][0] * in_x + alignment[2][1] * in_y +
         alignment[2][2] * in_z;
}

static bool write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, data, 2, 10);
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
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
     * that corrupted alternating 16-bit sensor words on the STM32F411.
     */
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_RESET);
    const HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, tx, rx, length + 1U, 10U);
    HAL_GPIO_WritePin(MPU6000_CS_PORT, MPU6000_CS_PIN, GPIO_PIN_SET);
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
    /* 96 MHz / 128 = 750 kHz for reset and configuration registers. */
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

    /* 96 MHz / 16 = 6 MHz for the 14-byte live sensor burst. */
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

    const float ax = (float)be16(&data[0]) / accel_lsb_per_g;
    const float ay = (float)be16(&data[2]) / accel_lsb_per_g;
    const float az = (float)be16(&data[4]) / accel_lsb_per_g;
    const float gx = (float)be16(&data[8]) / gyro_lsb_per_dps;
    const float gy = (float)be16(&data[10]) / gyro_lsb_per_dps;
    const float gz = (float)be16(&data[12]) / gyro_lsb_per_dps;
#if defined(BOARD_CLRACINGF4)
    /*
     * Betaflight's CLRACINGF4 target uses the MPU6000 with no board
     * alignment correction.  Its X/Y axes therefore already match the
     * flight-controller body frame.
     */
    sample->accel_x_g = -ax;
    sample->accel_y_g = ay;
    sample->gyro_x_dps = gx;
    sample->gyro_y_dps = -gy;
#else
    /* The MAMBAF411 MPU6000 is mounted CW180 around Z. */
    sample->accel_x_g = -ax;
    sample->accel_y_g = -ay;
    sample->gyro_x_dps = -gx;
    sample->gyro_y_dps = -gy;
#endif
    sample->accel_z_g = az;
    /*
     * Body yaw convention: rotating the nose clockwise/right when viewed
     * from above is positive.  The Mamba sensor Z axis has the opposite sign.
     */
    sample->gyro_z_dps = -gz;
    rotate(&sample->accel_x_g, &sample->accel_y_g, &sample->accel_z_g);
    rotate(&sample->gyro_x_dps, &sample->gyro_y_dps, &sample->gyro_z_dps);
    return true;
}
