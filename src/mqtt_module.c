#include "mqtt_module.h"
#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "mqtt_module";
static esp_mqtt_client_handle_t client;
bool full_stop = true;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "drone/commands", 0); // Suscribirse después de la conexión
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA: %.*s", event->data_len, event->data);
        if (strncmp(event->data, "s", event->data_len) == 0 && event->data_len == 1) // Verifica longitud correcta
        {
            full_stop = true;
        }
        else if (strncmp(event->data, "start", event->data_len) == 0 && event->data_len == 5) // Verifica longitud correcta
        {
            full_stop = false;
        }
        break;
    default:
        break;
    }
}

void init_mqtt(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000)); // Let the wifi connect
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.0.116:1883", // Actualiza con la IP de tu máquina Windows
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    ESP_LOGI(TAG, "MQTT client started");
}

void send_message(const char *message)
{
    int msg_id = esp_mqtt_client_publish(client, "drone/telemetry", message, 0, 1, 0);
    ESP_LOGI(TAG, "Message sent with ID: %d, message: %s", msg_id, message);
}
