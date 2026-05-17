#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "10Dof_IMU.h"
#include "dshot.h"
#include "drone_api.h"
#include "i2c_config.h"
#include "wifi_module.h"
#include "ws_server.h"

#define BLINK_GPIO       GPIO_NUM_2
#define TEST_THROTTLE    300
#define TEST_DURATION_MS 3000

static const char *TAG = "main";

typedef struct {
    float pitch, roll, yaw;
    float altitude;
    int motor[4];
} telemetry_t;

static telemetry_t       g_tlm;
static SemaphoreHandle_t g_tlm_mtx;
static volatile bool     g_full_stop = true;

typedef enum { MCMD_M1, MCMD_M2, MCMD_M3, MCMD_M4, MCMD_ALL } motor_cmd_t;

static QueueHandle_t g_cmd_q;

static const gpio_num_t MOTOR_PINS[4] = {
    ESC_GPIO_PIN_1, ESC_GPIO_PIN_2, ESC_GPIO_PIN_3, ESC_GPIO_PIN_4
};

static void all_motors_off(void) {
    for (int i = 0; i < 4; i++) {
        dshot_set_throttle(MOTOR_PINS[i], 0, false);
    }
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    for (int i = 0; i < 4; i++) g_tlm.motor[i] = 0;
    xSemaphoreGive(g_tlm_mtx);
}

bool api_get_full_stop(void) { return g_full_stop; }

void api_set_full_stop(bool v) {
    g_full_stop = v;
    if (v) all_motors_off();
    ESP_LOGW(TAG, "full_stop = %s", v ? "TRUE" : "FALSE");
}

static bool push_cmd(motor_cmd_t c) {
    if (g_full_stop) return false;
    return xQueueSend(g_cmd_q, &c, 0) == pdTRUE;
}

bool api_test_motor(int idx) {
    if (idx < 1 || idx > 4) return false;
    return push_cmd((motor_cmd_t)(idx - 1));
}

bool api_test_all(void) { return push_cmd(MCMD_ALL); }

void api_telemetry_json(char *buf, size_t n) {
    telemetry_t t;
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    t = g_tlm;
    xSemaphoreGive(g_tlm_mtx);
    snprintf(buf, n,
             "{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,\"alt\":%.2f,"
             "\"m\":[%d,%d,%d,%d],\"stop\":%s}",
             t.pitch, t.roll, t.yaw, t.altitude,
             t.motor[0], t.motor[1], t.motor[2], t.motor[3],
             g_full_stop ? "true" : "false");
}

static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void init_escs(void) {
    for (int i = 0; i < 4; i++) {
        dshot_config_t c = {
            .gpio_num = MOTOR_PINS[i],
            .type     = DSHOT300,
            .clk_src  = RMT_CLK_SRC_DEFAULT,
        };
        dshot_init(&c);
    }
    ESP_LOGI(TAG, "ESCs initialized, throttle=0");
    vTaskDelay(pdMS_TO_TICKS(3000));
}

static void set_motor(int i, int thr) {
    dshot_set_throttle(MOTOR_PINS[i], thr, false);
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    g_tlm.motor[i] = thr;
    xSemaphoreGive(g_tlm_mtx);
}

static void spin_motor(int i) {
    ESP_LOGI(TAG, "Spin M%d @ %d for %d ms", i + 1, TEST_THROTTLE, TEST_DURATION_MS);
    set_motor(i, TEST_THROTTLE);
    int waited = 0;
    while (waited < TEST_DURATION_MS) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
        if (g_full_stop) break;
    }
    set_motor(i, 0);
}

static void motor_task(void *p) {
    motor_cmd_t c;
    while (1) {
        if (xQueueReceive(g_cmd_q, &c, portMAX_DELAY) != pdTRUE) continue;
        if (g_full_stop) continue;
        if (c == MCMD_ALL) {
            for (int i = 0; i < 4 && !g_full_stop; i++) spin_motor(i);
        } else {
            spin_motor((int)c);
        }
    }
}

static void imu_task(void *p) {
    IMU_EN_SENSOR_TYPE m, prs;
    IMU_ST_ANGLES_DATA a;
    IMU_ST_SENSOR_DATA gy, ac, mg;
    int32_t T = 0, P = 0, alt = 0;
    imuInit(&m, &prs);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
        imuDataGet(&a, &gy, &ac, &mg);
        pressSensorDataGet(&T, &P, &alt);
        xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
        g_tlm.pitch    = -a.fPitch;
        g_tlm.roll     =  a.fRoll;
        g_tlm.yaw      =  a.fYaw;
        g_tlm.altitude = (float)alt / 100.0f;
        xSemaphoreGive(g_tlm_mtx);
    }
}

static void blink_task(void *p) {
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_log_level_set("IMU", ESP_LOG_WARN);
    esp_log_level_set("wifi", ESP_LOG_WARN);

    g_tlm_mtx = xSemaphoreCreateMutex();
    g_cmd_q   = xQueueCreate(8, sizeof(motor_cmd_t));

    i2c_master_init();
    init_escs();
    init_wifi();
    ws_server_start();

    xTaskCreate(blink_task, "blink", 1024, NULL, 5, NULL);
    xTaskCreate(imu_task,   "imu",   4096, NULL, 5, NULL);
    xTaskCreate(motor_task, "motor", 4096, NULL, 5, NULL);
}
