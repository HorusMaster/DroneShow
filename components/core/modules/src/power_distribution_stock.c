
#include <string.h>

#include "power_distribution.h"

#include <string.h>
#include "num.h"
#include "platform.h"
// #include "motors.h"
#include "esp_log.h"
#include "stabilizer_types.h"

// static const char *TAG = "PowerDistribution";

// Check why is False
static bool motorSetEnable = true;

motor_power_t motorPower;
// static struct {
//   uint16_t m1;
//   uint16_t m2;
//   uint16_t m3;
//   uint16_t m4;
// } motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
  // motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  // pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  // motorsSetRatio(MOTOR_M1, 0);
  // motorsSetRatio(MOTOR_M2, 0);
  // motorsSetRatio(MOTOR_M3, 0);
  // motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control, motor_power_t *motorPower)
{ 
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower->m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower->m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower->m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower->m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif 
  if (motorSetEnable)
  { 
   
    // motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    // motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    // motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    // motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    if (motorPower->m1 < idleThrust) {
      motorPower->m1 = idleThrust;
    }
    if (motorPower->m2 < idleThrust) {
      motorPower->m2 = idleThrust;
    }
    if (motorPower->m3 < idleThrust) {
      motorPower->m3 = idleThrust;
    }
    if (motorPower->m4 < idleThrust) {
      motorPower->m4 = idleThrust;
    }

    // motorsSetRatio(MOTOR_M1, motorPower.m1);
    // motorsSetRatio(MOTOR_M2, motorPower.m2);
    // motorsSetRatio(MOTOR_M3, motorPower.m3);
    // motorsSetRatio(MOTOR_M4, motorPower.m4);
  }
}
