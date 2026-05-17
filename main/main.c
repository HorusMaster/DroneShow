#include <math.h>
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
#include "nvs.h"
#include "nvs_flash.h"

#include "10Dof_IMU.h"
#include "dshot.h"
#include "drone_api.h"
#include "i2c_config.h"
#include "wifi_module.h"
#include "ws_server.h"

#define BLINK_GPIO        GPIO_NUM_2
#define TEST_THROTTLE     300
#define TEST_DURATION_MS  3000

#define CTRL_DT_MS        20
#define CTRL_DT_S         0.020f

#define THROTTLE_MAX      1000
#define MOTOR_MIN         0
#define MOTOR_MAX         1200

#define INTEGRAL_LIMIT    400.0f

static const char *TAG = "main";

typedef struct {
    float pitch, roll, yaw;
    float altitude;
    int   motor[4];
    float pid_out[3];
} telemetry_t;

static telemetry_t       g_tlm;
static SemaphoreHandle_t g_tlm_mtx;

static volatile bool g_full_stop = true;
static volatile bool g_armed     = false;
static volatile int  g_throttle  = 0;

static pid_gains_t       g_gains = { 2.0f, 0.0f, 0.0f, 0.5f };
static float             g_yaw_setpoint = 0.0f;
static float             g_pitch_offset = 0.0f;
static float             g_roll_offset  = 0.0f;
static SemaphoreHandle_t g_state_mtx;

typedef struct {
    float integral;
    float prev_error;
} pid_state_t;

static pid_state_t g_pid_p, g_pid_r, g_pid_y;

static volatile bool g_persist_dirty = false;
#define NVS_NS "drone"

typedef enum { MCMD_M1, MCMD_M2, MCMD_M3, MCMD_M4, MCMD_ALL } motor_cmd_t;
static QueueHandle_t g_cmd_q;

static const gpio_num_t MOTOR_PINS[4] = {
    ESC_GPIO_PIN_1, ESC_GPIO_PIN_2, ESC_GPIO_PIN_3, ESC_GPIO_PIN_4
};

static int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void set_motor(int i, int thr) {
    dshot_set_throttle(MOTOR_PINS[i], thr, false);
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    g_tlm.motor[i] = thr;
    xSemaphoreGive(g_tlm_mtx);
}

static void all_motors_off(void) {
    for (int i = 0; i < 4; i++) set_motor(i, 0);
}

static void pid_reset_all(void) {
    g_pid_p.integral = 0; g_pid_p.prev_error = 0;
    g_pid_r.integral = 0; g_pid_r.prev_error = 0;
    g_pid_y.integral = 0; g_pid_y.prev_error = 0;
}

static float pid_step(pid_state_t *s, float error, float kp, float ki, float kd, float dt) {
    s->integral = clampf(s->integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float derivative = (error - s->prev_error) / dt;
    s->prev_error = error;
    return kp * error + ki * s->integral + kd * derivative;
}

bool api_get_full_stop(void) { return g_full_stop; }

void api_set_full_stop(bool v) {
    g_full_stop = v;
    if (v) {
        g_armed = false;
        g_throttle = 0;
        all_motors_off();
        pid_reset_all();
    }
    ESP_LOGW(TAG, "full_stop = %s", v ? "TRUE" : "FALSE");
}

bool api_get_armed(void) { return g_armed; }

bool api_set_armed(bool v) {
    if (v) {
        if (g_full_stop) {
            ESP_LOGW(TAG, "ARM rechazado: full_stop activo");
            return false;
        }
        if (g_throttle != 0) {
            ESP_LOGW(TAG, "ARM rechazado: throttle != 0 (%d)", g_throttle);
            return false;
        }
        pid_reset_all();
        xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
        float yaw_now = g_tlm.yaw;
        xSemaphoreGive(g_tlm_mtx);
        xSemaphoreTake(g_state_mtx, portMAX_DELAY);
        g_yaw_setpoint = yaw_now;
        xSemaphoreGive(g_state_mtx);
        g_armed = true;
        ESP_LOGI(TAG, "ARMED. yaw_setpoint=%.1f", yaw_now);
    } else {
        g_armed = false;
        all_motors_off();
        pid_reset_all();
        ESP_LOGI(TAG, "DISARMED");
    }
    return true;
}

int api_get_throttle(void) { return g_throttle; }

void api_set_throttle(int v) {
    g_throttle = clampi(v, 0, THROTTLE_MAX);
}

void api_get_gains(pid_gains_t *out) {
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    *out = g_gains;
    xSemaphoreGive(g_state_mtx);
}

void api_set_gains(const pid_gains_t *in) {
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    g_gains = *in;
    xSemaphoreGive(g_state_mtx);
    g_persist_dirty = true;
}

static bool push_cmd(motor_cmd_t c) {
    if (g_full_stop || g_armed) return false;
    return xQueueSend(g_cmd_q, &c, 0) == pdTRUE;
}

bool api_test_motor(int idx) {
    if (idx < 1 || idx > 4) return false;
    return push_cmd((motor_cmd_t)(idx - 1));
}

bool api_test_all(void) { return push_cmd(MCMD_ALL); }

bool api_calibrate_level(void) {
    if (g_armed) {
        ESP_LOGW(TAG, "Calibrate rechazado: drone armado");
        return false;
    }
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    float p = g_tlm.pitch;
    float r = g_tlm.roll;
    xSemaphoreGive(g_tlm_mtx);
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    g_pitch_offset += p;
    g_roll_offset  += r;
    float new_po = g_pitch_offset;
    float new_ro = g_roll_offset;
    xSemaphoreGive(g_state_mtx);
    g_persist_dirty = true;
    ESP_LOGI(TAG, "Calibrate: pitch_off=%.2f roll_off=%.2f", new_po, new_ro);
    return true;
}

static void storage_load(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "NVS: namespace vacio, usando defaults");
        return;
    }
    pid_gains_t gains;
    size_t sz = sizeof(gains);
    if (nvs_get_blob(h, "gains", &gains, &sz) == ESP_OK && sz == sizeof(gains)) {
        xSemaphoreTake(g_state_mtx, portMAX_DELAY);
        g_gains = gains;
        xSemaphoreGive(g_state_mtx);
        ESP_LOGI(TAG, "NVS load gains: kP=%.2f kI=%.2f kD=%.2f kP_yaw=%.2f",
                 gains.kp_pr, gains.ki_pr, gains.kd_pr, gains.kp_yaw);
    }
    float offs[2];
    sz = sizeof(offs);
    if (nvs_get_blob(h, "offs", offs, &sz) == ESP_OK && sz == sizeof(offs)) {
        xSemaphoreTake(g_state_mtx, portMAX_DELAY);
        g_pitch_offset = offs[0];
        g_roll_offset  = offs[1];
        xSemaphoreGive(g_state_mtx);
        ESP_LOGI(TAG, "NVS load offsets: pitch=%.2f roll=%.2f", offs[0], offs[1]);
    }
    nvs_close(h);
}

static void storage_save(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    pid_gains_t gains;
    float offs[2];
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    gains   = g_gains;
    offs[0] = g_pitch_offset;
    offs[1] = g_roll_offset;
    xSemaphoreGive(g_state_mtx);
    nvs_set_blob(h, "gains", &gains, sizeof(gains));
    nvs_set_blob(h, "offs",  offs,   sizeof(offs));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "NVS save: gains + offsets");
}

static void storage_task(void *p) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1500));
        if (g_persist_dirty) {
            g_persist_dirty = false;
            storage_save();
        }
    }
}

void api_telemetry_json(char *buf, size_t n) {
    telemetry_t t;
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    t = g_tlm;
    xSemaphoreGive(g_tlm_mtx);

    pid_gains_t g;
    float yaw_sp;
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    g = g_gains;
    yaw_sp = g_yaw_setpoint;
    xSemaphoreGive(g_state_mtx);

    snprintf(buf, n,
             "{\"pitch\":%.2f,\"roll\":%.2f,\"yaw\":%.2f,\"alt\":%.2f,"
             "\"m\":[%d,%d,%d,%d],"
             "\"pid\":[%.1f,%.1f,%.1f],"
             "\"stop\":%s,\"armed\":%s,\"throttle\":%d,\"yaw_sp\":%.1f,"
             "\"gains\":{\"kp_pr\":%.2f,\"ki_pr\":%.2f,\"kd_pr\":%.2f,\"kp_yaw\":%.2f}}",
             t.pitch, t.roll, t.yaw, t.altitude,
             t.motor[0], t.motor[1], t.motor[2], t.motor[3],
             t.pid_out[0], t.pid_out[1], t.pid_out[2],
             g_full_stop ? "true" : "false",
             g_armed ? "true" : "false",
             g_throttle, yaw_sp,
             g.kp_pr, g.ki_pr, g.kd_pr, g.kp_yaw);
}

static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
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

static void spin_motor(int i) {
    ESP_LOGI(TAG, "Spin M%d @ %d for %d ms", i + 1, TEST_THROTTLE, TEST_DURATION_MS);
    set_motor(i, TEST_THROTTLE);
    int waited = 0;
    while (waited < TEST_DURATION_MS) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
        if (g_full_stop || g_armed) break;
    }
    set_motor(i, 0);
}

static void motor_task(void *p) {
    motor_cmd_t c;
    while (1) {
        if (xQueueReceive(g_cmd_q, &c, portMAX_DELAY) != pdTRUE) continue;
        if (g_full_stop || g_armed) continue;
        if (c == MCMD_ALL) {
            for (int i = 0; i < 4 && !g_full_stop && !g_armed; i++) spin_motor(i);
        } else {
            spin_motor((int)c);
        }
    }
}

static void control_step(void) {
    if (g_full_stop || !g_armed) {
        if (g_pid_p.integral != 0.0f || g_pid_r.integral != 0.0f ||
            g_pid_p.prev_error != 0.0f || g_pid_r.prev_error != 0.0f) {
            pid_reset_all();
        }
        xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
        g_tlm.pid_out[0] = g_tlm.pid_out[1] = g_tlm.pid_out[2] = 0.0f;
        xSemaphoreGive(g_tlm_mtx);
        return;
    }

    if (g_throttle <= 0) {
        pid_reset_all();
        for (int i = 0; i < 4; i++) set_motor(i, 0);
        xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
        g_tlm.pid_out[0] = g_tlm.pid_out[1] = g_tlm.pid_out[2] = 0.0f;
        xSemaphoreGive(g_tlm_mtx);
        return;
    }

    pid_gains_t g;
    float yaw_sp;
    xSemaphoreTake(g_state_mtx, portMAX_DELAY);
    g = g_gains;
    yaw_sp = g_yaw_setpoint;
    xSemaphoreGive(g_state_mtx);

    float pitch, roll, yaw;
    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    pitch = g_tlm.pitch;
    roll  = g_tlm.roll;
    yaw   = g_tlm.yaw;
    xSemaphoreGive(g_tlm_mtx);

    float yaw_err = yaw - yaw_sp;
    while (yaw_err > 180.0f)  yaw_err -= 360.0f;
    while (yaw_err < -180.0f) yaw_err += 360.0f;

    float out_p = pid_step(&g_pid_p, 0.0f - pitch, g.kp_pr, g.ki_pr, g.kd_pr, CTRL_DT_S);
    float out_r = pid_step(&g_pid_r, 0.0f - roll,  g.kp_pr, g.ki_pr, g.kd_pr, CTRL_DT_S);
    float out_y = g.kp_yaw * yaw_err;

    int base = g_throttle;
    int m1 = base + (int)out_p - (int)out_r + (int)out_y;
    int m2 = base - (int)out_p - (int)out_r - (int)out_y;
    int m3 = base - (int)out_p + (int)out_r + (int)out_y;
    int m4 = base + (int)out_p + (int)out_r - (int)out_y;

    set_motor(0, clampi(m1, MOTOR_MIN, MOTOR_MAX));
    set_motor(1, clampi(m2, MOTOR_MIN, MOTOR_MAX));
    set_motor(2, clampi(m3, MOTOR_MIN, MOTOR_MAX));
    set_motor(3, clampi(m4, MOTOR_MIN, MOTOR_MAX));

    xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
    g_tlm.pid_out[0] = out_p;
    g_tlm.pid_out[1] = out_r;
    g_tlm.pid_out[2] = out_y;
    xSemaphoreGive(g_tlm_mtx);
}

static void imu_task(void *p) {
    IMU_EN_SENSOR_TYPE m, prs;
    IMU_ST_ANGLES_DATA a;
    IMU_ST_SENSOR_DATA gy, ac, mg;
    int32_t T = 0, P = 0, alt = 0;
    imuInit(&m, &prs);

    TickType_t last = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CTRL_DT_MS));
        imuDataGet(&a, &gy, &ac, &mg);
        pressSensorDataGet(&T, &P, &alt);
        xSemaphoreTake(g_state_mtx, portMAX_DELAY);
        float po = g_pitch_offset;
        float ro = g_roll_offset;
        xSemaphoreGive(g_state_mtx);
        xSemaphoreTake(g_tlm_mtx, portMAX_DELAY);
        g_tlm.pitch    = -a.fPitch - po;
        g_tlm.roll     =  a.fRoll  - ro;
        g_tlm.yaw      =  a.fYaw;
        g_tlm.altitude = (float)alt / 100.0f;
        xSemaphoreGive(g_tlm_mtx);
        control_step();
    }
}

static void blink_task(void *p) {
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(g_armed ? 100 : 500));
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(g_armed ? 100 : 500));
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

    g_tlm_mtx   = xSemaphoreCreateMutex();
    g_state_mtx = xSemaphoreCreateMutex();
    g_cmd_q     = xQueueCreate(8, sizeof(motor_cmd_t));

    storage_load();

    i2c_master_init();
    init_escs();
    init_wifi();
    ws_server_start();

    xTaskCreate(blink_task,   "blink",   1024, NULL, 5, NULL);
    xTaskCreate(imu_task,     "imu",     4096, NULL, 6, NULL);
    xTaskCreate(motor_task,   "motor",   4096, NULL, 5, NULL);
    xTaskCreate(storage_task, "storage", 3072, NULL, 3, NULL);
}
