#ifndef DRONE_API_H
#define DRONE_API_H

#include <stdbool.h>
#include <stddef.h>

void api_telemetry_json(char *buf, size_t n);

bool api_get_full_stop(void);
void api_set_full_stop(bool v);

bool api_test_motor(int idx);
bool api_test_all(void);

#endif
