
#ifndef __SENSORS_ICM_20948_BMP_280_H__
#define __SENSORS_ICM_20948_BMP_280_H__

#include "sensors.h"

void sensorsMpu6050Hmc5883lMs5611Init(void);
bool sensorsMpu6050Hmc5883lMs5611Test(void);
bool sensorsMpu6050Hmc5883lMs5611AreCalibrated(void);
bool sensorsMpu6050Hmc5883lMs5611ManufacturingTest(void);
void sensorsMpu6050Hmc5883lMs5611Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu6050Hmc5883lMs5611WaitDataReady(void);
bool sensorsMpu6050Hmc5883lMs5611ReadGyro(Axis3f *gyro);
bool sensorsMpu6050Hmc5883lMs5611ReadAcc(Axis3f *acc);
bool sensorsMpu6050Hmc5883lMs5611ReadMag(Axis3f *mag);
bool sensorsMpu6050Hmc5883lMs5611ReadBaro(baro_t *baro);
void sensorsMpu6050Hmc5883lMs5611SetAccMode(accModes accMode);

#endif // __SENSORS_MPU9250_LPS25H_H__