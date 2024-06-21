#ifndef MQTT_MODULE_H
#define MQTT_MODULE_H

#include "stabilizer_types.h"

void init_mqtt(void);
void send_message(state_t* state);
#include <stdbool.h>
void set_full_stop(bool value);
bool get_full_stop(void);

void set_restart_escs(bool value);
bool get_restart_escs(void);
#endif // MQTT_MODULE_H
