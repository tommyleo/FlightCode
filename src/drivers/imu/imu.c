#include "imu.h"

#include <math.h>

#include "board.h"

#if BOARD_IMU_TYPE == IMU_TYPE_MPU6000
#include "mpu6000.h"
#elif BOARD_IMU_TYPE == IMU_TYPE_ICM42688P
#include "icm42688p.h"
#else
#error "Unsupported board IMU"
#endif

static float alignment[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

void imu_set_board_alignment(float roll_deg, float pitch_deg, float yaw_deg)
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

static void apply_common_sensor_axes(float *x, float *y, float *z)
{
    const float sensor_x = *x;
    const float sensor_y = *y;
    const float sensor_z = *z;

    /* Every supported IMU uses the same zero-yaw sensor-frame convention. */
    *x = sensor_x;
    *y = -sensor_y;
    *z = -sensor_z;
}

bool imu_init(uint32_t sample_rate_hz)
{
#if BOARD_IMU_TYPE == IMU_TYPE_MPU6000
    (void)sample_rate_hz;
    return mpu6000_init();
#else
    return icm42688p_init(sample_rate_hz);
#endif
}

const char *imu_get_name(void)
{
#if BOARD_IMU_TYPE == IMU_TYPE_MPU6000
    return "MPU6000";
#else
    return "ICM42688P";
#endif
}

bool imu_read(imu_sample_t *sample)
{
#if BOARD_IMU_TYPE == IMU_TYPE_MPU6000
    const bool read_ok = mpu6000_read(sample);
#else
    const bool read_ok = icm42688p_read(sample);
#endif
    if (!read_ok) {
        return false;
    }

    /*
     * Produce one canonical flight-controller frame:
     *   +roll  = right side down
     *   +pitch = nose up
     *   +yaw   = nose right
     * Accelerometer values represent the gravity vector, hence their global
     * sign reversal relative to each IMU's specific-force output.
     */
    apply_common_sensor_axes(&sample->accel_x_g,
                             &sample->accel_y_g,
                             &sample->accel_z_g);
    sample->accel_x_g = -sample->accel_x_g;
    sample->accel_y_g = -sample->accel_y_g;
    sample->accel_z_g = -sample->accel_z_g;
    apply_common_sensor_axes(&sample->gyro_x_dps,
                             &sample->gyro_y_dps,
                             &sample->gyro_z_dps);
    rotate(&sample->accel_x_g, &sample->accel_y_g, &sample->accel_z_g);
    rotate(&sample->gyro_x_dps, &sample->gyro_y_dps, &sample->gyro_z_dps);
    return true;
}
