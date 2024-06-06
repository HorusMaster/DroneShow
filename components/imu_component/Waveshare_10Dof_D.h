#ifndef __WAVESHARE_10DOF_D_H__
#define __WAVESHARE_10DOF_D_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "math.h"

/* Define ICM-20948 Device I2C address */
#define I2C_ADD_ICM20948 0x68
#define I2C_ADD_ICM20948_AK09916 0x0C
#define I2C_ADD_ICM20948_AK09916_READ 0x80
#define I2C_ADD_ICM20948_AK09916_WRITE 0x00

/* Define ICM-20948 Register */
/* User bank 0 register */
#define REG_ADD_WIA 0x00
#define REG_VAL_WIA 0xEA
#define REG_ADD_USER_CTRL 0x03
#define REG_VAL_BIT_DMP_EN 0x80
#define REG_VAL_BIT_FIFO_EN 0x40
#define REG_VAL_BIT_I2C_MST_EN 0x20
#define REG_VAL_BIT_I2C_IF_DIS 0x10
#define REG_VAL_BIT_DMP_RST 0x08
#define REG_VAL_BIT_DIAMOND_DMP_RST 0x04
#define REG_ADD_PWR_MIGMT_1 0x06
#define REG_VAL_ALL_RGE_RESET 0x80
#define REG_VAL_RUN_MODE 0x01
#define REG_ADD_LP_CONFIG 0x05
#define REG_ADD_PWR_MGMT_1 0x06
#define REG_ADD_PWR_MGMT_2 0x07
#define REG_ADD_ACCEL_XOUT_H 0x2D
#define REG_ADD_ACCEL_XOUT_L 0x2E
#define REG_ADD_ACCEL_YOUT_H 0x2F
#define REG_ADD_ACCEL_YOUT_L 0x30
#define REG_ADD_ACCEL_ZOUT_H 0x31
#define REG_ADD_ACCEL_ZOUT_L 0x32
#define REG_ADD_GYRO_XOUT_H 0x33
#define REG_ADD_GYRO_XOUT_L 0x34
#define REG_ADD_GYRO_YOUT_H 0x35
#define REG_ADD_GYRO_YOUT_L 0x36
#define REG_ADD_GYRO_ZOUT_H 0x37
#define REG_ADD_GYRO_ZOUT_L 0x38
#define REG_ADD_EXT_SENS_DATA_00 0x3B
#define REG_ADD_REG_BANK_SEL 0x7F
#define REG_VAL_REG_BANK_0 0x00
#define REG_VAL_REG_BANK_1 0x10
#define REG_VAL_REG_BANK_2 0x20
#define REG_VAL_REG_BANK_3 0x30

/* User bank 1 register */
/* User bank 2 register */
#define REG_ADD_GYRO_SMPLRT_DIV 0x00
#define REG_ADD_GYRO_CONFIG_1 0x01
#define REG_VAL_BIT_GYRO_DLPCFG_2 0x10
#define REG_VAL_BIT_GYRO_DLPCFG_4 0x20
#define REG_VAL_BIT_GYRO_DLPCFG_6 0x30
#define REG_VAL_BIT_GYRO_FS_250DPS 0x00
#define REG_VAL_BIT_GYRO_FS_500DPS 0x02
#define REG_VAL_BIT_GYRO_FS_1000DPS 0x04
#define REG_VAL_BIT_GYRO_FS_2000DPS 0x06
#define REG_VAL_BIT_GYRO_DLPF 0x01
#define REG_ADD_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ADD_ACCEL_CONFIG 0x14
#define REG_VAL_BIT_ACCEL_DLPCFG_2 0x10
#define REG_VAL_BIT_ACCEL_DLPCFG_4 0x20
#define REG_VAL_BIT_ACCEL_DLPCFG_6 0x30
#define REG_VAL_BIT_ACCEL_FS_2g 0x00
#define REG_VAL_BIT_ACCEL_FS_4g 0x02
#define REG_VAL_BIT_ACCEL_FS_8g 0x04
#define REG_VAL_BIT_ACCEL_FS_16g 0x06
#define REG_VAL_BIT_ACCEL_DLPF 0x01

/* User bank 3 register */
#define REG_ADD_I2C_SLV0_ADDR 0x03
#define REG_ADD_I2C_SLV0_REG 0x04
#define REG_ADD_I2C_SLV0_CTRL 0x05
#define REG_VAL_BIT_SLV0_EN 0x80
#define REG_VAL_BIT_MASK_LEN 0x07
#define REG_ADD_I2C_SLV0_DO 0x06
#define REG_ADD_I2C_SLV1_ADDR 0x07
#define REG_ADD_I2C_SLV1_REG 0x08
#define REG_ADD_I2C_SLV1_CTRL 0x09
#define REG_ADD_I2C_SLV1_DO 0x0A

/* Define ICM-20948 MAG Register */
#define REG_ADD_MAG_WIA1 0x00
#define REG_VAL_MAG_WIA1 0x48
#define REG_ADD_MAG_WIA2 0x01
#define REG_VAL_MAG_WIA2 0x09
#define REG_ADD_MAG_ST2 0x10
#define REG_ADD_MAG_DATA 0x11
#define REG_ADD_MAG_CNTL2 0x31
#define REG_VAL_MAG_MODE_PD 0x00
#define REG_VAL_MAG_MODE_SM 0x01
#define REG_VAL_MAG_MODE_10HZ 0x02
#define REG_VAL_MAG_MODE_20HZ 0x04
#define REG_VAL_MAG_MODE_50HZ 0x05
#define REG_VAL_MAG_MODE_100HZ 0x08
#define REG_VAL_MAG_MODE_ST 0x10

#define MAG_DATA_LEN 6

/******************************************************************************
 * BMP280 sensor device                                                       *
 ******************************************************************************/
#define BMP280_REGISTER_DIG_T1 0x88
#define BMP280_REGISTER_DIG_T2 0x8A
#define BMP280_REGISTER_DIG_T3 0x8C

#define BMP280_REGISTER_DIG_P1 0x8E
#define BMP280_REGISTER_DIG_P2 0x90
#define BMP280_REGISTER_DIG_P3 0x92
#define BMP280_REGISTER_DIG_P4 0x94
#define BMP280_REGISTER_DIG_P5 0x96
#define BMP280_REGISTER_DIG_P6 0x98
#define BMP280_REGISTER_DIG_P7 0x9A
#define BMP280_REGISTER_DIG_P8 0x9C
#define BMP280_REGISTER_DIG_P9 0x9E

#define BMP280_REGISTER_CHIPID 0xD0
#define BMP280_REGISTER_VERSION 0xD1
#define BMP280_REGISTER_SOFTRESET 0xE0
#define BMP280_REGISTER_STATUS 0xF3
#define BMP280_REGISTER_CONTROL 0xF4
#define BMP280_REGISTER_CONFIG 0xF5

#define BMP280_TEMP_XLSB_REG 0xFC  /*Temperature XLSB Register */
#define BMP280_TEMP_LSB_REG 0xFB   /*Temperature LSB Register  */
#define BMP280_TEMP_MSB_REG 0xFA   /*Temperature LSB Register  */
#define BMP280_PRESS_XLSB_REG 0xF9 /*Pressure XLSB  Register   */
#define BMP280_PRESS_LSB_REG 0xF8  /*Pressure LSB Register     */
#define BMP280_PRESS_MSB_REG 0xF7  /*Pressure MSB Register     */

/*calibration parameters */
#define BMP280_DIG_T1_LSB_REG 0x88
#define BMP280_DIG_T1_MSB_REG 0x89
#define BMP280_DIG_T2_LSB_REG 0x8A
#define BMP280_DIG_T2_MSB_REG 0x8B
#define BMP280_DIG_T3_LSB_REG 0x8C
#define BMP280_DIG_T3_MSB_REG 0x8D
#define BMP280_DIG_P1_LSB_REG 0x8E
#define BMP280_DIG_P1_MSB_REG 0x8F
#define BMP280_DIG_P2_LSB_REG 0x90
#define BMP280_DIG_P2_MSB_REG 0x91
#define BMP280_DIG_P3_LSB_REG 0x92
#define BMP280_DIG_P3_MSB_REG 0x93
#define BMP280_DIG_P4_LSB_REG 0x94
#define BMP280_DIG_P4_MSB_REG 0x95
#define BMP280_DIG_P5_LSB_REG 0x96
#define BMP280_DIG_P5_MSB_REG 0x97
#define BMP280_DIG_P6_LSB_REG 0x98
#define BMP280_DIG_P6_MSB_REG 0x99
#define BMP280_DIG_P7_LSB_REG 0x9A
#define BMP280_DIG_P7_MSB_REG 0x9B
#define BMP280_DIG_P8_LSB_REG 0x9C
#define BMP280_DIG_P8_MSB_REG 0x9D
#define BMP280_DIG_P9_LSB_REG 0x9E
#define BMP280_DIG_P9_MSB_REG 0x9F

typedef struct
{
    uint16_t T1;    /*<calibration T1 data*/
    int16_t T2;     /*<calibration T2 data*/
    int16_t T3;     /*<calibration T3 data*/
    uint16_t P1;    /*<calibration P1 data*/
    int16_t P2;     /*<calibration P2 data*/
    int16_t P3;     /*<calibration P3 data*/
    int16_t P4;     /*<calibration P4 data*/
    int16_t P5;     /*<calibration P5 data*/
    int16_t P6;     /*<calibration P6 data*/
    int16_t P7;     /*<calibration P7 data*/
    int16_t P8;     /*<calibration P8 data*/
    int16_t P9;     /*<calibration P9 data*/
    int32_t T_fine; /*<calibration t_fine data*/
} BMP280_HandleTypeDef;

typedef struct
{
    uint8_t Index;
    int32_t AvgBuffer[8];
} BMP280_AvgTypeDef;

#define dig_T1 bmp280.T1
#define dig_T2 bmp280.T2
#define dig_T3 bmp280.T3
#define dig_P1 bmp280.P1
#define dig_P2 bmp280.P2
#define dig_P3 bmp280.P3
#define dig_P4 bmp280.P4
#define dig_P5 bmp280.P5
#define dig_P6 bmp280.P6
#define dig_P7 bmp280.P7
#define dig_P8 bmp280.P8
#define dig_P9 bmp280.P9
#define t_fine bmp280.T_fine

#define MSLP 101325 // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)


// int32_t gs32Pressure0 = MSLP;

typedef enum
{
    IMU_EN_SENSOR_TYPE_NULL = 0,
    IMU_EN_SENSOR_TYPE_ICM20948,
    IMU_EN_SENSOR_TYPE_BMP280,
    IMU_EN_SENSOR_TYPE_MAX
} IMU_EN_SENSOR_TYPE;

typedef struct imu_st_angles_data_tag
{
    float fYaw;
    float fPitch;
    float fRoll;
} IMU_ST_ANGLES_DATA;

typedef struct imu_st_sensor_data_tag
{
    int16_t s16X;
    int16_t s16Y;
    int16_t s16Z;
} IMU_ST_SENSOR_DATA;

typedef struct icm20948_st_avg_data_tag
{
    uint8_t u8Index;
    int16_t s16AvgBuffer[8];
} ICM20948_ST_AVG_DATA;

uint8_t i2c_read_byte(uint8_t dev_addr, uint8_t reg_addr);
void i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

void imuInit(IMU_EN_SENSOR_TYPE *penMotionSensorType, IMU_EN_SENSOR_TYPE *penPressureType);
void imuDataGet(IMU_ST_ANGLES_DATA *pstAngles,
                IMU_ST_SENSOR_DATA *pstGyroRawData,
                IMU_ST_SENSOR_DATA *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData);
void pressSensorDataGet(int32_t *ps32Temperature, int32_t *ps32Pressure, int32_t *ps32Altitude);

void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);

void icm20948init(void);
bool icm20948Check(void);
void icm20948GyroRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z);
void icm20948AccelRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z);
void icm20948MagRead(int16_t *ps16X, int16_t *ps16Y, int16_t *ps16Z);
bool icm20948MagCheck(void);
void icm20948CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int32_t *pOutVal);
void icm20948GyroOffset(void);
void icm20948ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data);
void icm20948WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data);

bool bmp280Check(void);
void bmp280Init(void);
float bmp280CompensateTemperature(int32_t adc_T);
float bmp280CompensatePressure(int32_t adc_P);
void bmp280TandPGet(float *temperature, float *pressure);
void bmp280CalAvgValue(uint8_t *pIndex, int32_t *pAvgBuffer, int32_t InVal, int32_t *pOutVal);
void bmp280CalculateAbsoluteAltitude(int32_t *pAltitude, int32_t PressureVal);

#ifdef __cplusplus
}
#endif

#endif // __WAVESHARE_10DOF_D_H__
