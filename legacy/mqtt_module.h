#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

void init_mqtt(void);
void send_message(const char* message);
#include <stdbool.h>
void set_full_stop(bool value);
bool get_full_stop(void);

void set_restart_escs(bool value);
bool get_restart_escs(void);
#endif // MQTT_MODULE_H
