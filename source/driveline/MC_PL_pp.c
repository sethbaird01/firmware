#include "MC_PL_pp.h"
#include "MC_PL0.h"
#include "can_parse.h"
#include "wheel_speeds.h"

void MC_PL_pp(ExtU* rtU) {
    rtU->Too[0] = 0;
    rtU->Too[1] = 0;
    rtU->Too[2] = can_data.torque_request_main.rear_left + 0.1;
    rtU->Too[3] = can_data.torque_request_main.rear_right + 0.1;
    rtU->Woo[0] = 0;
    rtU->Woo[1] = 0;
    rtU->Woo[2] = wheel_speeds.left_kph_x100 * SPEED_CALIBRATION;
    //can_data.rear_wheel_data.left_speed * SPEED_CALIBRATION;
    rtU->Woo[3] = wheel_speeds.right_kph_x100 * SPEED_CALIBRATION;
    //can_data.rear_wheel_data.right_speed * SPEED_CALIBRATION;
    //rtU->Vbatt = can_data.orion_currents_volts.pack_voltage * VOLTAGE_CALIBRATION;
    rtU->Vbatt = 200;
    //rtU->Pmax = can_data.orion_info.discharge_enable;
    //rtU->Pmin = can_data.orion_info.charge_enable;

    rtU->Pmax = 10000;
    rtU->Pmin = 0;
}