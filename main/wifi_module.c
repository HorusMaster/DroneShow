#include "wifi_module.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h" // Asegúrate de incluir esto para eventos Wi-Fi

static const char *TAG = "wifi_module";

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    static int retry_count = 0;
    const int max_retries = 5; // Número máximo de reintentos

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (retry_count < max_retries)
        {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Retrying to connect to the AP");
        }
        else
        {
            ESP_LOGI(TAG, "Failed to connect to the AP after %d attempts", max_retries);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        retry_count = 0; // Reinicia el contador de reintentos al obtener una IP
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip[IP4ADDR_STRLEN_MAX];
        esp_ip4addr_ntoa(&event->ip_info.ip, ip, IP4ADDR_STRLEN_MAX);
        ESP_LOGI(TAG, "Got IP: %s", ip);
    }
}

void init_wifi(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi");

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t sta_cfg = {
        .sta = {
            .ssid = "NASA-Wi-Fi",
            .password = "valle200@",
            .listen_interval = 10, // Incrementa el intervalo de escucha para mejorar la reconexión
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Deshabilita el modo de ahorro de energía
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78)); // Establece la potencia de transmisión al máximo permitido

    ESP_LOGI(TAG, "Wi-Fi initialized and connected");
}
