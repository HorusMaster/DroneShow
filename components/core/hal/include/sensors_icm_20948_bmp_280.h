
#ifndef __SENSORS_ICM_20948_BMP_280_H__
#define __SENSORS_ICM_20948_BMP_280_H__

#include "sensors.h"

void sensorsICM20948BMP280Init(void);
bool sensorsICM20948BMP280Test(void);
bool sensorsICM20948BMP280AreCalibrated(void);
bool sensorsICM20948BMP280ManufacturingTest(void);
void sensorsICM20948BMP280Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsICM20948BMP280WaitDataReady(void);
bool sensorsICM20948BMP280ReadGyro(Axis3f *gyro);
bool sensorsICM20948BMP280ReadAcc(Axis3f *acc);
bool sensorsICM20948BMP280ReadMag(Axis3f *mag);
bool sensorsICM20948BMP280ReadBaro(baro_t *baro);
void sensorsICM20948BMP280SetAccMode(accModes accMode);

#endif // __SENSORS_ICM_20948_BMP_280_H__