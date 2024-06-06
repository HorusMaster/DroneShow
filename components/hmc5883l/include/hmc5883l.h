#ifndef HMC5883L_H
#define HMC5883L_H

#include "driver/i2c.h"
#include "i2c_config.h"

#define HMC5883L_DEVICE_ADDR 0x3C

#define HMC5883L_CRA 0x00
#define HMC5883L_CRB 0x01
#define HMC5883L_MR 0x02
#define HMC5883L_DOX_H 0x03
#define HMC5883L_DOX_L 0x04
#define HMC5883L_DOZ_H 0x05
#define HMC5883L_DOZ_L 0x06
#define HMC5883L_DOY_H 0x07
#define HMC5883L_DOY_L 0x08

#define HMC5883L_SAMPLE_AVERAGE 0x03
#define HMC5883L_DOR 0x06
#define HMC5883L_MM 0x00
#define HMC5883L_GAIN 0x05
#define HMC5883L_CMM 0x00

typedef struct {
    uint8_t Index;
    int16_t AvgBuffer[8];
} HMC5883L_AvgTypeDef;

void HMC5883L_Init(void);
void HMC5883L_DataOutRegister(int16_t *pData);
void HMC5883L_Compass(int16_t MagAngleX, int16_t MagAngleY, float *Angle);

#endif // HMC5883L_H
