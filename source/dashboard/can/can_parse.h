/**
 * @file can_parse.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 *
 * @copyright Copyright (c) 2021
 *
 */
#ifndef _CAN_PARSE_H_
#define _CAN_PARSE_H_

#include "common/queue/queue.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/can/can.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "Dashboard"

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_RAW_THROTTLE_BRAKE 0x14000285
#define ID_COOLING_DRIVER_REQUEST 0xc0002c5
#define ID_FILT_THROTTLE_BRAKE 0x4000245
#define ID_START_BUTTON 0x4000005
#define ID_DASHBOARD_HB 0x4001905
#define ID_DASHBOARD_VOLTS_TEMP 0x4001945
#define ID_DASHBOARD_TV_PARAMETERS 0x4000dc5
#define ID_FAULT_SYNC_DASHBOARD 0x8cac5
#define ID_DAQ_RESPONSE_DASHBOARD 0x17ffffc5
#define ID_MAIN_HB 0x4001901
#define ID_REAR_MOTOR_CURRENTS_TEMPS 0xc0002c1
#define ID_ORION_INFO 0x140006b8
#define ID_ORION_CURRENTS_VOLTS 0x140006f8
#define ID_ORION_ERRORS 0xc000738
#define ID_MAX_CELL_TEMP 0x404e604
#define ID_REAR_CONTROLLER_TEMPS 0xc000301
#define ID_PRECHARGE_HB 0x4001944
#define ID_TORQUE_REQUEST_MAIN 0x4000041
#define ID_REAR_WHEEL_SPEEDS 0x8000381
#define ID_COOLANT_TEMPS 0x4000881
#define ID_COOLANT_OUT 0x40008df
#define ID_GEARBOX 0x10000901
#define ID_DASHBOARD_BL_CMD 0x409c47e
#define ID_FAULT_SYNC_PDU 0x8cb1f
#define ID_FAULT_SYNC_MAIN_MODULE 0x8ca01
#define ID_FAULT_SYNC_A_BOX 0x8ca44
#define ID_FAULT_SYNC_TORQUE_VECTOR 0x8cab7
#define ID_FAULT_SYNC_TEST_NODE 0x8cb7f
#define ID_SET_FAULT 0x809c83e
#define ID_RETURN_FAULT_CONTROL 0x809c87e
#define ID_DAQ_COMMAND_DASHBOARD 0x14000172
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_RAW_THROTTLE_BRAKE 8
#define DLC_COOLING_DRIVER_REQUEST 5
#define DLC_FILT_THROTTLE_BRAKE 3
#define DLC_START_BUTTON 1
#define DLC_DASHBOARD_HB 1
#define DLC_DASHBOARD_VOLTS_TEMP 6
#define DLC_DASHBOARD_TV_PARAMETERS 7
#define DLC_FAULT_SYNC_DASHBOARD 3
#define DLC_DAQ_RESPONSE_DASHBOARD 8
#define DLC_MAIN_HB 2
#define DLC_REAR_MOTOR_CURRENTS_TEMPS 8
#define DLC_ORION_INFO 7
#define DLC_ORION_CURRENTS_VOLTS 4
#define DLC_ORION_ERRORS 4
#define DLC_MAX_CELL_TEMP 2
#define DLC_REAR_CONTROLLER_TEMPS 2
#define DLC_PRECHARGE_HB 2
#define DLC_TORQUE_REQUEST_MAIN 8
#define DLC_REAR_WHEEL_SPEEDS 8
#define DLC_COOLANT_TEMPS 4
#define DLC_COOLANT_OUT 3
#define DLC_GEARBOX 2
#define DLC_DASHBOARD_BL_CMD 5
#define DLC_FAULT_SYNC_PDU 3
#define DLC_FAULT_SYNC_MAIN_MODULE 3
#define DLC_FAULT_SYNC_A_BOX 3
#define DLC_FAULT_SYNC_TORQUE_VECTOR 3
#define DLC_FAULT_SYNC_TEST_NODE 3
#define DLC_SET_FAULT 3
#define DLC_RETURN_FAULT_CONTROL 2
#define DLC_DAQ_COMMAND_DASHBOARD 8
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_RAW_THROTTLE_BRAKE(queue, throttle_, throttle_right_, brake_, brake_right_, brake_pot_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_RAW_THROTTLE_BRAKE, .DLC=DLC_RAW_THROTTLE_BRAKE, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->raw_throttle_brake.throttle = throttle_;\
        data_a->raw_throttle_brake.throttle_right = throttle_right_;\
        data_a->raw_throttle_brake.brake = brake_;\
        data_a->raw_throttle_brake.brake_right = brake_right_;\
        data_a->raw_throttle_brake.brake_pot = brake_pot_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_COOLING_DRIVER_REQUEST(queue, dt_pump_, dt_fan_, batt_pump_, batt_pump2_, batt_fan_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_COOLING_DRIVER_REQUEST, .DLC=DLC_COOLING_DRIVER_REQUEST, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->cooling_driver_request.dt_pump = dt_pump_;\
        data_a->cooling_driver_request.dt_fan = dt_fan_;\
        data_a->cooling_driver_request.batt_pump = batt_pump_;\
        data_a->cooling_driver_request.batt_pump2 = batt_pump2_;\
        data_a->cooling_driver_request.batt_fan = batt_fan_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_FILT_THROTTLE_BRAKE(queue, throttle_, brake_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_FILT_THROTTLE_BRAKE, .DLC=DLC_FILT_THROTTLE_BRAKE, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->filt_throttle_brake.throttle = throttle_;\
        data_a->filt_throttle_brake.brake = brake_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_START_BUTTON(queue, start_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_START_BUTTON, .DLC=DLC_START_BUTTON, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->start_button.start = start_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_DASHBOARD_HB(queue, apps_faulted_, bse_faulted_, apps_brake_faulted_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_DASHBOARD_HB, .DLC=DLC_DASHBOARD_HB, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->dashboard_hb.apps_faulted = apps_faulted_;\
        data_a->dashboard_hb.bse_faulted = bse_faulted_;\
        data_a->dashboard_hb.apps_brake_faulted = apps_brake_faulted_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_DASHBOARD_VOLTS_TEMP(queue, mcu_temp_, volts_5v_, volts_3v3_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_DASHBOARD_VOLTS_TEMP, .DLC=DLC_DASHBOARD_VOLTS_TEMP, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->dashboard_volts_temp.mcu_temp = mcu_temp_;\
        data_a->dashboard_volts_temp.volts_5v = volts_5v_;\
        data_a->dashboard_volts_temp.volts_3v3 = volts_3v3_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_DASHBOARD_TV_PARAMETERS(queue, tv_enabled_, tv_deadband_val_, tv_intensity_val_, tv_p_val_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_DASHBOARD_TV_PARAMETERS, .DLC=DLC_DASHBOARD_TV_PARAMETERS, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->dashboard_tv_parameters.tv_enabled = tv_enabled_;\
        data_a->dashboard_tv_parameters.tv_deadband_val = tv_deadband_val_;\
        data_a->dashboard_tv_parameters.tv_intensity_val = tv_intensity_val_;\
        data_a->dashboard_tv_parameters.tv_p_val = tv_p_val_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_FAULT_SYNC_DASHBOARD(queue, idx_, latched_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_FAULT_SYNC_DASHBOARD, .DLC=DLC_FAULT_SYNC_DASHBOARD, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->fault_sync_dashboard.idx = idx_;\
        data_a->fault_sync_dashboard.latched = latched_;\
        qSendToBack(&queue, &msg);\
    } while(0)
#define SEND_DAQ_RESPONSE_DASHBOARD(queue, daq_response_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_DAQ_RESPONSE_DASHBOARD, .DLC=DLC_DAQ_RESPONSE_DASHBOARD, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->daq_response_DASHBOARD.daq_response = daq_response_;\
        qSendToBack(&queue, &msg);\
    } while(0)
/* END AUTO SEND MACROS */

// Stale Checking
#define STALE_THRESH 3 / 2 // 3 / 2 would be 150% of period
/* BEGIN AUTO UP DEFS (Update Period)*/
#define UP_MAIN_HB 100
#define UP_REAR_MOTOR_CURRENTS_TEMPS 500
#define UP_ORION_INFO 32
#define UP_ORION_CURRENTS_VOLTS 32
#define UP_ORION_ERRORS 1000
#define UP_MAX_CELL_TEMP 500
#define UP_REAR_CONTROLLER_TEMPS 500
#define UP_PRECHARGE_HB 100
#define UP_TORQUE_REQUEST_MAIN 15
#define UP_REAR_WHEEL_SPEEDS 15
#define UP_COOLANT_TEMPS 200
#define UP_COOLANT_OUT 1000
#define UP_GEARBOX 2000
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) > period * STALE_THRESH) stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
typedef enum {
    CAR_STATE_IDLE,
    CAR_STATE_PRECHARGING,
    CAR_STATE_ENERGIZED,
    CAR_STATE_BUZZING,
    CAR_STATE_READY2DRIVE,
    CAR_STATE_ERROR,
    CAR_STATE_FATAL,
    CAR_STATE_RESET,
    CAR_STATE_RECOVER,
    CAR_STATE_FAN_CTRL,
} car_state_t;

/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { 
    struct {
        uint64_t throttle: 12;
        uint64_t throttle_right: 12;
        uint64_t brake: 12;
        uint64_t brake_right: 12;
        uint64_t brake_pot: 12;
    } raw_throttle_brake;
    struct {
        uint64_t dt_pump: 8;
        uint64_t dt_fan: 8;
        uint64_t batt_pump: 8;
        uint64_t batt_pump2: 8;
        uint64_t batt_fan: 8;
    } cooling_driver_request;
    struct {
        uint64_t throttle: 12;
        uint64_t brake: 12;
    } filt_throttle_brake;
    struct {
        uint64_t start: 1;
    } start_button;
    struct {
        uint64_t apps_faulted: 1;
        uint64_t bse_faulted: 1;
        uint64_t apps_brake_faulted: 1;
    } dashboard_hb;
    struct {
        uint64_t mcu_temp: 16;
        uint64_t volts_5v: 16;
        uint64_t volts_3v3: 16;
    } dashboard_volts_temp;
    struct {
        uint64_t tv_enabled: 1;
        uint64_t tv_deadband_val: 16;
        uint64_t tv_intensity_val: 16;
        uint64_t tv_p_val: 16;
    } dashboard_tv_parameters;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_dashboard;
    struct {
        uint64_t daq_response: 64;
    } daq_response_DASHBOARD;
    struct {
        uint64_t car_state: 8;
        uint64_t precharge_state: 1;
    } main_hb;
    struct {
        uint64_t left_current: 16;
        uint64_t right_current: 16;
        uint64_t left_temp: 8;
        uint64_t right_temp: 8;
        uint64_t right_voltage: 16;
    } rear_motor_currents_temps;
    struct {
        uint64_t discharge_enable: 1;
        uint64_t charge_enable: 1;
        uint64_t charger_safety: 1;
        uint64_t dtc_status: 1;
        uint64_t multi_input: 1;
        uint64_t always_on: 1;
        uint64_t is_ready: 1;
        uint64_t is_charging: 1;
        uint64_t multi_input_2: 1;
        uint64_t multi_input_3: 1;
        uint64_t reserved: 1;
        uint64_t multi_output_2: 1;
        uint64_t multi_output_3: 1;
        uint64_t multi_output_4: 1;
        uint64_t multi_enable: 1;
        uint64_t multi_output_1: 1;
        uint64_t pack_dcl: 16;
        uint64_t pack_ccl: 16;
        uint64_t pack_soc: 8;
    } orion_info;
    struct {
        uint64_t pack_current: 16;
        uint64_t pack_voltage: 16;
    } orion_currents_volts;
    struct {
        uint64_t discharge_limit_enforce: 1;
        uint64_t charger_safety_relay: 1;
        uint64_t internal_hardware: 1;
        uint64_t heatsink_thermistor: 1;
        uint64_t software: 1;
        uint64_t max_cellv_high: 1;
        uint64_t min_cellv_low: 1;
        uint64_t pack_overheat: 1;
        uint64_t reserved0: 1;
        uint64_t reserved1: 1;
        uint64_t reserved2: 1;
        uint64_t reserved3: 1;
        uint64_t reserved4: 1;
        uint64_t reserved5: 1;
        uint64_t reserved6: 1;
        uint64_t reserved7: 1;
        uint64_t internal_comms: 1;
        uint64_t cell_balancing_foff: 1;
        uint64_t weak_cell: 1;
        uint64_t low_cellv: 1;
        uint64_t open_wire: 1;
        uint64_t current_sensor: 1;
        uint64_t max_cellv_o5v: 1;
        uint64_t cell_asic: 1;
        uint64_t weak_pack: 1;
        uint64_t fan_monitor: 1;
        uint64_t thermistor: 1;
        uint64_t external_comms: 1;
        uint64_t redundant_psu: 1;
        uint64_t hv_isolation: 1;
        uint64_t input_psu: 1;
        uint64_t charge_limit_enforce: 1;
    } orion_errors;
    struct {
        uint64_t max_temp: 16;
    } max_cell_temp;
    struct {
        uint64_t left_temp: 8;
        uint64_t right_temp: 8;
    } rear_controller_temps;
    struct {
        uint64_t IMD: 8;
        uint64_t BMS: 8;
    } precharge_hb;
    struct {
        uint64_t front_left: 16;
        uint64_t front_right: 16;
        uint64_t rear_left: 16;
        uint64_t rear_right: 16;
    } torque_request_main;
    struct {
        uint64_t left_speed_mc: 16;
        uint64_t right_speed_mc: 16;
        uint64_t left_speed_sensor: 16;
        uint64_t right_speed_sensor: 16;
    } rear_wheel_speeds;
    struct {
        uint64_t battery_in_temp: 8;
        uint64_t battery_out_temp: 8;
        uint64_t drivetrain_in_temp: 8;
        uint64_t drivetrain_out_temp: 8;
    } coolant_temps;
    struct {
        uint64_t bat_fan: 8;
        uint64_t dt_fan: 8;
        uint64_t bat_pump: 1;
        uint64_t bat_pump_aux: 1;
        uint64_t dt_pump: 1;
    } coolant_out;
    struct {
        uint64_t l_temp: 8;
        uint64_t r_temp: 8;
    } gearbox;
    struct {
        uint64_t cmd: 8;
        uint64_t data: 32;
    } dashboard_bl_cmd;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_pdu;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_main_module;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_a_box;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_torque_vector;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_test_node;
    struct {
        uint64_t id: 16;
        uint64_t value: 1;
    } set_fault;
    struct {
        uint64_t id: 16;
    } return_fault_control;
    struct {
        uint64_t daq_command: 64;
    } daq_command_DASHBOARD;
    uint8_t raw_data[8];
} __attribute__((packed)) CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        car_state_t car_state;
        uint8_t precharge_state;
        uint8_t stale;
        uint32_t last_rx;
    } main_hb;
    struct {
        uint16_t left_current;
        uint16_t right_current;
        uint8_t left_temp;
        uint8_t right_temp;
        uint16_t right_voltage;
        uint8_t stale;
        uint32_t last_rx;
    } rear_motor_currents_temps;
    struct {
        uint8_t discharge_enable;
        uint8_t charge_enable;
        uint8_t charger_safety;
        uint8_t dtc_status;
        uint8_t multi_input;
        uint8_t always_on;
        uint8_t is_ready;
        uint8_t is_charging;
        uint8_t multi_input_2;
        uint8_t multi_input_3;
        uint8_t reserved;
        uint8_t multi_output_2;
        uint8_t multi_output_3;
        uint8_t multi_output_4;
        uint8_t multi_enable;
        uint8_t multi_output_1;
        uint16_t pack_dcl;
        uint16_t pack_ccl;
        uint8_t pack_soc;
        uint8_t stale;
        uint32_t last_rx;
    } orion_info;
    struct {
        int16_t pack_current;
        uint16_t pack_voltage;
        uint8_t stale;
        uint32_t last_rx;
    } orion_currents_volts;
    struct {
        uint8_t discharge_limit_enforce;
        uint8_t charger_safety_relay;
        uint8_t internal_hardware;
        uint8_t heatsink_thermistor;
        uint8_t software;
        uint8_t max_cellv_high;
        uint8_t min_cellv_low;
        uint8_t pack_overheat;
        uint8_t reserved0;
        uint8_t reserved1;
        uint8_t reserved2;
        uint8_t reserved3;
        uint8_t reserved4;
        uint8_t reserved5;
        uint8_t reserved6;
        uint8_t reserved7;
        uint8_t internal_comms;
        uint8_t cell_balancing_foff;
        uint8_t weak_cell;
        uint8_t low_cellv;
        uint8_t open_wire;
        uint8_t current_sensor;
        uint8_t max_cellv_o5v;
        uint8_t cell_asic;
        uint8_t weak_pack;
        uint8_t fan_monitor;
        uint8_t thermistor;
        uint8_t external_comms;
        uint8_t redundant_psu;
        uint8_t hv_isolation;
        uint8_t input_psu;
        uint8_t charge_limit_enforce;
        uint8_t stale;
        uint32_t last_rx;
    } orion_errors;
    struct {
        int16_t max_temp;
        uint8_t stale;
        uint32_t last_rx;
    } max_cell_temp;
    struct {
        uint8_t left_temp;
        uint8_t right_temp;
        uint8_t stale;
        uint32_t last_rx;
    } rear_controller_temps;
    struct {
        uint8_t IMD;
        uint8_t BMS;
        uint8_t stale;
        uint32_t last_rx;
    } precharge_hb;
    struct {
        int16_t front_left;
        int16_t front_right;
        int16_t rear_left;
        int16_t rear_right;
        uint8_t stale;
        uint32_t last_rx;
    } torque_request_main;
    struct {
        uint16_t left_speed_mc;
        uint16_t right_speed_mc;
        uint16_t left_speed_sensor;
        uint16_t right_speed_sensor;
        uint8_t stale;
        uint32_t last_rx;
    } rear_wheel_speeds;
    struct {
        int8_t battery_in_temp;
        int8_t battery_out_temp;
        int8_t drivetrain_in_temp;
        int8_t drivetrain_out_temp;
        uint8_t stale;
        uint32_t last_rx;
    } coolant_temps;
    struct {
        uint8_t bat_fan;
        uint8_t dt_fan;
        uint8_t bat_pump;
        uint8_t bat_pump_aux;
        uint8_t dt_pump;
        uint8_t stale;
        uint32_t last_rx;
    } coolant_out;
    struct {
        int8_t l_temp;
        int8_t r_temp;
        uint8_t stale;
        uint32_t last_rx;
    } gearbox;
    struct {
        uint8_t cmd;
        uint32_t data;
    } dashboard_bl_cmd;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_pdu;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_main_module;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_a_box;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_torque_vector;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_test_node;
    struct {
        uint16_t id;
        uint8_t value;
    } set_fault;
    struct {
        uint16_t id;
    } return_fault_control;
    struct {
        uint64_t daq_command;
    } daq_command_DASHBOARD;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void daq_command_DASHBOARD_CALLBACK(CanMsgTypeDef_t* msg_header_a);
extern void coolant_out_CALLBACK(CanParsedData_t* msg_data_a);
extern void dashboard_bl_cmd_CALLBACK(CanParsedData_t* msg_data_a);
extern void handleCallbacks(uint16_t id, bool latched);
extern void set_fault_daq(uint16_t id, bool value);
extern void return_fault_control(uint16_t id);
extern void send_fault(uint16_t id, bool latched);
/* END AUTO EXTERN CALLBACK */

/* BEGIN AUTO EXTERN RX IRQ */
/* END AUTO EXTERN RX IRQ */

/**
 * @brief Setup queue and message filtering
 *
 * @param q_rx_can RX buffer of CAN messages
 */
void initCANParse(q_handle_t* q_rx_can_a);

/**
 * @brief Pull message off of rx buffer,
 *        update can_data struct,
 *        check for stale messages
 */
void canRxUpdate();

/**
 * @brief Process any rx message callbacks from the CAN Rx IRQ
 *
 * @param rx rx data from message just recieved
 */
void canProcessRxIRQs(CanMsgTypeDef_t* rx);

extern volatile uint32_t last_can_rx_time_ms;

#endif