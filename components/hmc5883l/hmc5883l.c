#include "hmc5883l.h"
#include "driver/i2c.h"
#include <math.h>
#include "i2c_config.h"
#include "esp_log.h"

static const char *TAG = "HMC5883L";

static esp_err_t i2c_master_write_byte_custom(i2c_port_t i2c_num, uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Write Failed: addr=0x%02X, reg=0x%02X, data=0x%02X", device_addr, reg_addr, data);
    }
    return ret;
}

static esp_err_t i2c_master_read_custom(i2c_port_t i2c_num, uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Write Failed: addr=0x%02X, reg=0x%02X", device_addr, reg_addr);
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Read Failed: addr=0x%02X, reg=0x%02X", device_addr, reg_addr);
    }
    return ret;
}

void HMC5883L_WriteReg(uint8_t RegAddr, uint8_t Data) {
    if (i2c_master_write_byte_custom(I2C_MASTER_NUM, HMC5883L_DEVICE_ADDR, RegAddr, Data) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write to register %02X", RegAddr);
    }
}

void HMC5883L_Init(void) {
    HMC5883L_WriteReg(HMC5883L_CRA, (HMC5883L_SAMPLE_AVERAGE << 5) | (HMC5883L_DOR << 2) | HMC5883L_MM);
    HMC5883L_WriteReg(HMC5883L_CRB, (HMC5883L_GAIN << 5) & 0xE0);
    HMC5883L_WriteReg(HMC5883L_MR, HMC5883L_CMM);
    ESP_LOGI(TAG, "HMC5883L initialized");
}

void HMC5883L_CalAvgValue(uint8_t *pIndex, int16_t *pAvgBuffer, int16_t InVal, int16_t *pOutVal) {
    uint8_t i;

    *(pAvgBuffer + ((*pIndex)++)) = InVal;
    *pIndex &= 0x07;

    *pOutVal = 0;
    for (i = 0; i < 8; i++) {
        *pOutVal += *(pAvgBuffer + i);
    }
    *pOutVal >>= 3;
}

void HMC5883L_DataOutRegister(int16_t *pData) {
    uint8_t Buffer[6] = {0};
    int16_t AvgBuff[3] = {0};
    static HMC5883L_AvgTypeDef HMC5883L_Filter[3];

    if (NULL == pData) {
        ESP_LOGE(TAG, "Null pointer provided for data output");
        return;
    }

    if (i2c_master_read_custom(I2C_MASTER_NUM, HMC5883L_DEVICE_ADDR, HMC5883L_DOX_H, Buffer, 6) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data output register");
        return;
    }

    HMC5883L_CalAvgValue(&HMC5883L_Filter[0].Index, HMC5883L_Filter[0].AvgBuffer, ((int16_t)Buffer[0] << 8) | Buffer[1], (int16_t *)AvgBuff);
    HMC5883L_CalAvgValue(&HMC5883L_Filter[1].Index, HMC5883L_Filter[1].AvgBuffer, ((int16_t)Buffer[4] << 8) | Buffer[5], (int16_t *)AvgBuff + 1);
    HMC5883L_CalAvgValue(&HMC5883L_Filter[2].Index, HMC5883L_Filter[2].AvgBuffer, ((int16_t)Buffer[2] << 8) | Buffer[3], (int16_t *)AvgBuff + 2);

    *pData = *AvgBuff;
    *(pData + 1) = *(AvgBuff + 1);
    *(pData + 2) = *(AvgBuff + 2);
}

void HMC5883L_Compass(int16_t MagAngleX, int16_t MagAngleY, float *Angle) {
    *Angle = atan2(MagAngleY, MagAngleX) * (180 / 3.14159265);
}
