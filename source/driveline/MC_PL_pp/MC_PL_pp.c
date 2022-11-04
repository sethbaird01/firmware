#include "MC_PL_pp.h"
#include "MC_PL0.h"
#include "can_parse.h"
#include "wheel_speeds.h"
#include "common_defs.h"

void MC_PL_pp(ExtU* rtU) {
    rtU->Too[0] = CLAMP(0.0, ABS_MIN_TORQUE, ABS_MAX_TORQUE);
    rtU->Too[1] = CLAMP(0.0, ABS_MIN_TORQUE, ABS_MAX_TORQUE);
    rtU->Too[2] = CLAMP(can_data.torque_request_main.rear_left * TORQUE_CALIBRATION, ABS_MIN_TORQUE, ABS_MAX_TORQUE);
    rtU->Too[3] = CLAMP(can_data.torque_request_main.rear_right * TORQUE_CALIBRATION, ABS_MIN_TORQUE, ABS_MAX_TORQUE);

    rtU->Woo[0] = CLAMP(0.0, MIN_OMEGA, MAX_OMEGA);
    rtU->Woo[1] = CLAMP(0.0, MIN_OMEGA, MAX_OMEGA);
    rtU->Woo[2] = CLAMP(wheel_speeds.left_kph_x100 * SPEED_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
    rtU->Woo[3] = CLAMP(wheel_speeds.right_kph_x100 * SPEED_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
    //can_data.rear_wheel_data.left_speed * SPEED_CALIBRATION;
    //can_data.rear_wheel_data.right_speed * SPEED_CALIBRATION;

    //rtU->Vbatt = CLAMP(can_data.orion_currents_volts.pack_voltage * VOLTAGE_CALIBRATION, MIN_VOLTAGE, MAX_VOLTAGE);
    rtU->Vbatt = CLAMP(200.0, MIN_VOLTAGE, MAX_VOLTAGE);

    //rtU->Pmax = CLAMP((rtU->battery_voltage) * can_data.orion_info.pack_dcl, 0, MAX_BATTERY_POWER);
    //rtU->Pmin = CLAMP((rtU->battery_voltage) * can_data.orion_info.pack_ccl, 0, MIN_BATTERY_POWER);
    rtU->Pmax = CLAMP(10000.0, 0.0, MAX_BATTERY_POWER);
    rtU->Pmin = CLAMP(0.0, 0.0, MIN_BATTERY_POWER);
}