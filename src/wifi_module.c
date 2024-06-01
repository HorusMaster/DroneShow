#include "wifi_module.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h" // Asegúrate de incluir esto para eventos Wi-Fi

static const char *TAG = "wifi_module";

void init_wifi(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi");

    // Initialize Wi-Fi
    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t sta_cfg = {
        .sta = {
            .ssid = "NASA-Wi-Fi",
            .password = "valle200@",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, "Wi-Fi initialized and connected");
}
