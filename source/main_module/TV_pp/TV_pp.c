#include "TV_pp.h"
#include "ELectronics.h"
#include "can_parse.h"

void TV_pp(ExtU* rtU)
{
rtU->r_ref = 0;

rtU->driver_input = can_data.raw_throttle_brake.throttle * DRIVER_INPUT_CALIBRATION;

rtU->steering_angle = can_data.LWS_Standard.LWS_ANGLE * STEERING_ANGLE_CALIBRATION;

rtU->brake_pressure[0] = 0;
rtU->brake_pressure[1] = 0;
rtU->brake_pressure[2] = 0;
rtU->brake_pressure[3] = 0;

//rtU->ang[0] = can_data.angle_data.pitch * ANGLE_CALIBRATION;
//rtU->ang[1] = can_data.angle_data.roll * ANGLE_CALIBRATION;
//rtU->ang[2] = can_data.angle_data.yaw * ANGLE_CALIBRATION;

rtU->ang[0] = 0 * ANGLE_CALIBRATION;
rtU->ang[1] = 0 * ANGLE_CALIBRATION;
rtU->ang[2] = 0 * ANGLE_CALIBRATION;

rtU->ang_vel[0] = can_data.gyro_data.gx * ROTATION_CALIBRATION;
rtU->ang_vel[1] = can_data.gyro_data.gy * ROTATION_CALIBRATION;
rtU->ang_vel[2] = can_data.gyro_data.gz * ROTATION_CALIBRATION;

rtU->accel[0] = can_data.accel_data.ax * ACCELERATION_CALIBRATION;
rtU->accel[1] = can_data.accel_data.ay * ACCELERATION_CALIBRATION;
rtU->accel[2] = can_data.accel_data.az * ACCELERATION_CALIBRATION;

rtU->omega[0] = 0.1;
rtU->omega[1] = 0.1;
rtU->omega[2] = can_data.rear_wheel_data.left_speed * OMEGA_CALIBRATION;
rtU->omega[3] = can_data.rear_wheel_data.right_speed * OMEGA_CALIBRATION;

if (rtU->omega[2] < 0.1){
    rtU->omega[2] = 0.1;
} else {
    rtU->omega[3] = 0.1;
}
    


rtU->vel[0] = (rtU->omega[2] + rtU->omega[3]) * RE / 2;
rtU->vel[1] = 0;

rtU->shock_displacement[0] = 0;
rtU->shock_displacement[1] = 0;
rtU->shock_displacement[2] = can_data.rear_wheel_data.left_normal * SHOCK_CALIBRATION;
rtU->shock_displacement[3] = can_data.rear_wheel_data.right_normal * SHOCK_CALIBRATION;

rtU->shock_velocity[0] = 0;
rtU->shock_velocity[1] = 0;
rtU->shock_velocity[2] = 0;
rtU->shock_velocity[3] = 0;

rtU->motor_temp[0] = 55;
rtU->motor_temp[1] = 55;
rtU->motor_temp[2] = 55;
rtU->motor_temp[3] = 55;

//rtU->battery_voltage = can_data.orion_currents_volts.pack_voltage * VOLTAGE_CALIBRATION;

rtU->battery_voltage = 200;

//rtU->power_limits[0] = (rtU->battery_voltage) * can_data.orion_info.pack_dcl * CURRENT_CALIBRATION;
//rtU->power_limits[1] = (rtU->battery_voltage) * can_data.orion_info.pack_ccl * CURRENT_CALIBRATION;

rtU->power_limits[0] = 10000;
rtU->power_limits[1] = 10000;

// Temporary
rtU->FZ[0] = 1200;
rtU->FZ[1] = 1200;
rtU->FZ[2] = 1200;
rtU->FZ[3] = 1200;
}