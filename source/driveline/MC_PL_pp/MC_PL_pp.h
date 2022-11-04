#include "MC_PL0.h"

// Calibration
#define VOLTAGE_CALIBRATION 1.0
#define SPEED_CALIBRATION (1.0/100.0) * 0.277778
#define TORQUE_CALIBRATION 25.0 / 4095.0

// Clamping
#define ABS_MIN_TORQUE -0.01
#define ABS_MAX_TORQUE 25.0

#define MIN_OMEGA 0.0
#define MAX_OMEGA 1070.0

#define MIN_VOLTAGE 50
#define MAX_VOLTAGE 340

#define MIN_BATTERY_POWER -80000.0
#define MAX_BATTERY_POWER 80000.0

void MC_PL_pp(ExtU* rtU);