#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "imu.h"
#define IMU_TYPE_MPU6000 1
#define IMU_TYPE_ICM42688P 2
#define IMU_TYPE_AUTODETECT 3
#define BOARD_IMU_TYPE IMU_TYPE_AUTODETECT
static unsigned fitted,selected,requested_rate;
static bool mpu6000_init(void){return fitted==1U;}
static bool icm42688p_init(uint32_t rate){requested_rate=rate;return fitted==2U;}
static uint32_t mpu6000_get_gyro_rate_hz(void){return 8000U;}
static uint32_t icm42688p_get_gyro_rate_hz(void){return requested_rate;}
static void board_imu_select(uint8_t candidate){selected=candidate;}
static bool sensor_read(imu_sample_t *s){*s=(imu_sample_t){.gyro_x_dps=1,.gyro_y_dps=2,.gyro_z_dps=3,.accel_x_g=1,.accel_y_g=2,.accel_z_g=3};return true;}
static bool mpu6000_read(imu_sample_t *s){return sensor_read(s);}
static bool icm42688p_read(imu_sample_t *s){return sensor_read(s);}
#include "foxeer_imu_under_test.inc"

int main(void)
{
    for(fitted=1U;fitted<=2U;fitted++){
        assert(imu_init(16000U));assert(selected==0U);
        assert(strcmp(imu_get_name(),fitted==1U?"MPU6000":"ICM42688P")==0);
        assert(imu_get_gyro_rate_hz()==(fitted==1U?8000U:16000U));
        assert(imu_gyro_rate_supported(8000U));
        assert(imu_gyro_rate_supported(16000U)==(fitted==2U));
        assert(!imu_gyro_rate_supported(32000U));
        imu_sample_t s;
        imu_set_board_alignment(0,0,-90);assert(imu_read(&s));
        assert(fabsf(s.gyro_x_dps+2)<0.001f&&fabsf(s.gyro_y_dps+1)<0.001f&&fabsf(s.gyro_z_dps+3)<0.001f);
        imu_set_board_alignment(0,0,0);assert(imu_read(&s));
        assert(s.gyro_x_dps==1&&s.gyro_y_dps==-2&&s.gyro_z_dps==-3);
    }
    fitted=0U;assert(!imu_init(16000U));assert(selected==1U);
    assert(strcmp(imu_get_name(),"Not detected")==0);
    assert(!imu_gyro_rate_supported(16000U));
    puts("Foxeer IMU autodetection, rates, missing sensor and F722/H743 axes passed");
}
