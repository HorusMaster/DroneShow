#include "dshot.h"
#include "driver/rmt_tx.h"
#include "dshot_esc_encoder.h"
#include "esp_log.h"

static const char *TAG = "dshot";

static const uint32_t DSHOT_ESC_RESOLUTION_HZ = 40000000; // 40MHz resolution
static rmt_channel_handle_t esc_chans[4] = { NULL, NULL, NULL, NULL };
static rmt_encoder_handle_t dshot_encoders[4] = { NULL, NULL, NULL, NULL };

static int get_channel_index(gpio_num_t gpio_num) {
    switch (gpio_num) {
        case ESC_GPIO_PIN_1:
            return 0;
        case ESC_GPIO_PIN_2:
            return 1;
        case ESC_GPIO_PIN_3:
            return 2;
        case ESC_GPIO_PIN_4:
            return 3;
        default:
            return -1; // Invalid channel
    }
}

esp_err_t dshot_init(const dshot_config_t *config) {
    int index = get_channel_index(config->gpio_num);
    if (index == -1) {
        ESP_LOGE(TAG, "Invalid GPIO pin");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Create RMT TX channel for GPIO %d", config->gpio_num);
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = config->clk_src,
        .gpio_num = config->gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chans[index]));

    ESP_LOGI(TAG, "Install Dshot ESC encoder for GPIO %d", config->gpio_num);
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = config->type * 1000,
        .post_delay_us = 50,
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoders[index]));

    ESP_LOGI(TAG, "Enable RMT TX channel for GPIO %d", config->gpio_num);
    ESP_ERROR_CHECK(rmt_enable(esc_chans[index]));

    return ESP_OK;
}

void dshot_set_throttle(gpio_num_t gpio_num, uint16_t throttle, bool telemetry) {
    int index = get_channel_index(gpio_num);
    if (index == -1) {
        ESP_LOGE(TAG, "Invalid GPIO pin");
        return;
    }

    dshot_esc_throttle_t throttle_data = {
        .throttle = throttle,
        .telemetry_req = telemetry,
    };
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(esc_chans[index], dshot_encoders[index], &throttle_data, sizeof(throttle_data), &tx_config));
}


void dshot_set_throttle2(gpio_num_t gpio_num, uint16_t throttle, bool telemetry) {
    int index = get_channel_index(gpio_num);
    if (index == -1) {
        ESP_LOGE(TAG, "Invalid GPIO pin");
        return;
    }

    dshot_esc_throttle_t throttle_data = {
        .throttle = 0,
        .telemetry_req = telemetry,
    };
    rmt_transmit_config_t tx_config = {
        .loop_count = -1,
    };

    ESP_ERROR_CHECK(rmt_transmit(esc_chans[index], dshot_encoders[index], &throttle_data, sizeof(throttle_data), &tx_config));
}