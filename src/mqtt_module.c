#include "mqtt_module.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_module";
static esp_mqtt_client_handle_t client;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    default:
        break;
    }
}

void init_mqtt(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://172.29.106.73:1883",  // Updated to the new format
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));

    ESP_LOGI(TAG, "MQTT client started");
}

void send_message(const char* message)
{
    esp_mqtt_client_publish(client, "drone/lights", message, 0, 1, 0);
    ESP_LOGI(TAG, "Message sent: %s", message);
}
