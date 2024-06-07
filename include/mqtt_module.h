#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

void init_mqtt(void);
void send_message(const char* message);
#include <stdbool.h>
extern bool full_stop;
#endif // MQTT_MODULE_H
