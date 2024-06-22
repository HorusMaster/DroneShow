#include "mqtt_module.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "stabilizer_types.h"

static const char *TAG = "mqtt_module";
static esp_mqtt_client_handle_t client;
static bool full_stop = true;
static bool restart_escs = false;
static char message[512];

void set_full_stop(bool value)
{
    full_stop = value;
}

bool get_full_stop(void)
{
    return full_stop;
}

void set_restart_escs(bool value)
{
    restart_escs = value;
}

bool get_restart_escs(void)
{
    return restart_escs;
}

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
        // ESP_LOGI(TAG, "MQTT_EVENT_DATA: %.*s", event->data_len, event->data);
        if (strncmp(event->data, "s", event->data_len) == 0 && event->data_len == 1) // Verifica longitud correcta
        {
            ESP_LOGI(TAG, "Stopping ESCS");
            set_full_stop(true);
        }
        else if (strncmp(event->data, "start", event->data_len) == 0 && event->data_len == 5) // Verifica longitud correcta
        {
            ESP_LOGI(TAG, "Starting ESCS");
            set_full_stop(false);
        }
        else if (strncmp(event->data, "restart", event->data_len) == 0 && event->data_len == 7) // Verifica longitud correcta
        {
            ESP_LOGI(TAG, "Restarting ESCS");
            set_restart_escs(true);
        }
        break;
    default:
        break;
    }
}

void init_mqtt(void)
{
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Let the wifi connect
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.0.116:1883", // Actualiza con la IP de tu máquina Windows
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
    ESP_LOGI(TAG, "MQTT client started");
}

void send_message(state_t *state, control_t *control, motor_power_t *motorPower)
{
    snprintf(message, sizeof(message),
             "{\"pitch\": %.2f, \"roll\": %.2f, \"yaw\": %.2f, \"altitude\": %.2f, "
             "\"motor1\": %.2f, \"motor2\": %.2f, \"motor3\": %.2f, \"motor4\": %.2f, "
             "\"pidpitch\": %.2f, \"pidroll\": %.2f, \"pidyaw\": %.2f, \"pidalt\": %.2f}",
             state->attitude.pitch, state->attitude.roll, state->attitude.yaw, 0.0,
             (double)motorPower->m1, (double)motorPower->m2, (double)motorPower->m3, (double)motorPower->m4,
             (double)control->pitch, (double)control->roll, (double)control->yaw, (double)control->thrust);
    esp_mqtt_client_publish(client, "drone/telemetry", message, 0, 1, 0);
}
