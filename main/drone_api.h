#ifndef DRONE_API_H
#define DRONE_API_H

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    float kp_pr;
    float ki_pr;
    float kd_pr;
    float kp_yaw;
} pid_gains_t;

void api_telemetry_json(char *buf, size_t n);

bool api_get_full_stop(void);
void api_set_full_stop(bool v);

bool api_get_armed(void);
bool api_set_armed(bool v);

int  api_get_throttle(void);
void api_set_throttle(int v);

void api_get_gains(pid_gains_t *out);
void api_set_gains(const pid_gains_t *in);

bool api_test_motor(int idx);
bool api_test_all(void);

bool api_calibrate_level(void);

#endif
