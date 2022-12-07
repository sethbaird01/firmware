#include "TV_pp.h"
#include "ELectronics.h"
#include "can_parse.h"
#include "common_defs.h"

void TV_pp(ExtU* rtU)
{
rtU->r_ref = CLAMP(200, MIN_R_REF, MAX_R_REF);

rtU->driver_input = CLAMP(can_data.raw_throttle_brake.throttle * DRIVER_INPUT_CALIBRATION, MIN_THROTTLE, MAX_THROTTLE);

rtU->steering_angle = CLAMP(can_data.LWS_Standard.LWS_ANGLE * STEERING_ANGLE_CALIBRATION, MIN_STEERING, MAX_STEERING);

//rtU->ang[0] = CLAMP(can_data.angle_data.pitch * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
//rtU->ang[1] = CLAMP(can_data.angle_data.roll * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
//rtU->ang[2] = CLAMP(can_data.angle_data.yaw * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);

rtU->ang_vel[0] = CLAMP(can_data.gyro_data.gx * ROTATION_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);
rtU->ang_vel[1] = CLAMP(can_data.gyro_data.gy * ROTATION_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);
rtU->ang_vel[2] = CLAMP(can_data.gyro_data.gz * ROTATION_CALIBRATION, MIN_ANG_VEL, MAX_ANG_VEL);

rtU->accel[0] = CLAMP(can_data.accel_data.ax * ACCELERATION_CALIBRATION, MIN_ACCEL, MAX_ACCEL);
rtU->accel[1] = CLAMP(can_data.accel_data.ay * ACCELERATION_CALIBRATION, MIN_ACCEL, MAX_ACCEL);
rtU->accel[2] = CLAMP(can_data.accel_data.az * ACCELERATION_CALIBRATION, MIN_ACCEL, MAX_ACCEL);

rtU->omega[0] = CLAMP(0.0 * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
rtU->omega[1] = CLAMP(0.0 * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
rtU->omega[2] = CLAMP(can_data.rear_wheel_data.left_speed * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);
rtU->omega[3] = CLAMP(can_data.rear_wheel_data.right_speed * OMEGA_CALIBRATION, MIN_OMEGA, MAX_OMEGA);

rtU->vel[0] = CLAMP((rtU->omega[2] + rtU->omega[3]) * 0.222 / 2, MIN_VEL, MAX_VEL);
rtU->vel[1] = CLAMP(0.0, MIN_VEL, MAX_VEL);

rtU->shock_displacement[0] = CLAMP(0.0, MIN_SHOCK_D, MAX_SHOCK_D);
rtU->shock_displacement[1] = CLAMP(0.0, MIN_SHOCK_D, MAX_SHOCK_D);
rtU->shock_displacement[2] = CLAMP(can_data.rear_wheel_data.left_normal * SHOCK_CALIBRATION, MIN_SHOCK_D, MAX_SHOCK_D);
rtU->shock_displacement[3] = CLAMP(can_data.rear_wheel_data.right_normal * SHOCK_CALIBRATION, MIN_SHOCK_D, MAX_SHOCK_D);

rtU->shock_velocity[0] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);
rtU->shock_velocity[1] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);
rtU->shock_velocity[2] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);
rtU->shock_velocity[3] = CLAMP(0.0, MIN_SHOCK_V, MAX_SHOCK_V);

rtU->motor_temperature[0] = CLAMP(55.0, MIN_MOTOR_T, MAX_MOTOR_T);
rtU->motor_temperature[1] = CLAMP(55.0, MIN_MOTOR_T, MAX_MOTOR_T);
rtU->motor_temperature[2] = CLAMP(55.0, MIN_MOTOR_T, MAX_MOTOR_T);
rtU->motor_temperature[3] = CLAMP(55.0, MIN_MOTOR_T, MAX_MOTOR_T);

//rtU->battery_voltage = CLAMP(can_data.orion_currents_volts.pack_voltage * VOLTAGE_CALIBRATION, MIN_VOLTAGE, MAX_VOLTAGE);

//rtU->power_limits[0] = CLAMP(can_data.orion_currents_volts.pack_voltage * VOLTAGE_CALIBRATION * can_data.orion_info.pack_dcl * CURRENT_CALIBRATION, 0, MAX_BATTERY_POWER);
//rtU->power_limits[1] = CLAMP(can_data.orion_currents_volts.pack_voltage * VOLTAGE_CALIBRATION * can_data.orion_info.pack_ccl * CURRENT_CALIBRATION, 0, MIN_BATTERY_POWER);

// Temporary
rtU->FZ[0] = CLAMP(1200.0, MIN_FZ, MAX_FZ);
rtU->FZ[1] = CLAMP(1200.0, MIN_FZ, MAX_FZ);
rtU->FZ[2] = CLAMP(1200.0, MIN_FZ, MAX_FZ);
rtU->FZ[3] = CLAMP(1200.0, MIN_FZ, MAX_FZ);

rtU->ang[0] = CLAMP(0.0 * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
rtU->ang[1] = CLAMP(0.0 * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);
rtU->ang[2] = CLAMP(0.0 * ANGLE_CALIBRATION, MIN_ANG, MAX_ANG);

rtU->battery_voltage = CLAMP(300.0, MIN_VOLTAGE, MAX_VOLTAGE);

rtU->power_limits[0] = CLAMP(80000.0, 0.0, MAX_BATTERY_POWER);
rtU->power_limits[1] = CLAMP(80000.0, 0.0, MIN_BATTERY_POWER);
}