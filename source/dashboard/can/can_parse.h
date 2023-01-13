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
#include "common/phal_L4/can/can.h"

// Make this match the node name within the can_config.json
#define NODE_NAME "Dashboard"

// Message ID definitions
/* BEGIN AUTO ID DEFS */
#define ID_RAW_THROTTLE_BRAKE 0x14000285
#define ID_START_BUTTON 0x4000005
#define ID_DASHBOARD_HB 0x4001905
#define ID_FAULT_SYNC_DASHBOARD 0xc001e45
#define ID_DAQ_RESPONSE_DASHBOARD 0x17ffffc5
#define ID_FAULT_SYNC_DRIVELINE 0xc001e83
#define ID_FAULT_SYNC_TORQUE_VECTOR 0xc001e82
#define ID_FAULT_SYNC_MAIN_MODULE 0xc001e41
#define ID_FAULT_SYNC_PRECHARGE 0xc001e84
#define ID_FAULT_SYNC_L4_TESTING 0xc001eff
#define ID_SET_FAULT 0x809c83e
#define ID_RETURN_FAULT_CONTROL 0x809c87e
#define ID_MAIN_HB 0x4001901
#define ID_FRONT_WHEEL_DATA 0x4000003
#define ID_DAQ_COMMAND_DASHBOARD 0x14000172
/* END AUTO ID DEFS */

// Message DLC definitions
/* BEGIN AUTO DLC DEFS */
#define DLC_RAW_THROTTLE_BRAKE 3
#define DLC_START_BUTTON 1
#define DLC_DASHBOARD_HB 1
#define DLC_FAULT_SYNC_DASHBOARD 3
#define DLC_DAQ_RESPONSE_DASHBOARD 8
#define DLC_FAULT_SYNC_DRIVELINE 3
#define DLC_FAULT_SYNC_TORQUE_VECTOR 3
#define DLC_FAULT_SYNC_MAIN_MODULE 3
#define DLC_FAULT_SYNC_PRECHARGE 3
#define DLC_FAULT_SYNC_L4_TESTING 3
#define DLC_SET_FAULT 3
#define DLC_RETURN_FAULT_CONTROL 2
#define DLC_MAIN_HB 2
#define DLC_FRONT_WHEEL_DATA 8
#define DLC_DAQ_COMMAND_DASHBOARD 8
/* END AUTO DLC DEFS */

// Message sending macros
/* BEGIN AUTO SEND MACROS */
#define SEND_RAW_THROTTLE_BRAKE(queue, throttle_, brake_) do {\
        CanMsgTypeDef_t msg = {.Bus=CAN1, .ExtId=ID_RAW_THROTTLE_BRAKE, .DLC=DLC_RAW_THROTTLE_BRAKE, .IDE=1};\
        CanParsedData_t* data_a = (CanParsedData_t *) &msg.Data;\
        data_a->raw_throttle_brake.throttle = throttle_;\
        data_a->raw_throttle_brake.brake = brake_;\
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
#define UP_FRONT_WHEEL_DATA 10
/* END AUTO UP DEFS */

#define CHECK_STALE(stale, curr, last, period) if(!stale && \
                    (curr - last) > period * STALE_THRESH) stale = 1

/* BEGIN AUTO CAN ENUMERATIONS */
typedef enum {
    CAR_STATE_INIT,
    CAR_STATE_BUZZING,
    CAR_STATE_READY2DRIVE,
    CAR_STATE_ERROR,
    CAR_STATE_FATAL,
    CAR_STATE_RESET,
    CAR_STATE_RECOVER,
} car_state_t;

/* END AUTO CAN ENUMERATIONS */

// Message Raw Structures
/* BEGIN AUTO MESSAGE STRUCTURE */
typedef union { __attribute__((packed))
    struct {
        uint64_t throttle: 12;
        uint64_t brake: 12;
    } raw_throttle_brake;
    struct {
        uint64_t start: 1;
    } start_button;
    struct {
        uint64_t apps_faulted: 1;
        uint64_t bse_faulted: 1;
        uint64_t apps_brake_faulted: 1;
    } dashboard_hb;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_dashboard;
    struct {
        uint64_t daq_response: 64;
    } daq_response_DASHBOARD;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_driveline;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_torque_vector;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_main_module;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_precharge;
    struct {
        uint64_t idx: 16;
        uint64_t latched: 1;
    } fault_sync_l4_testing;
    struct {
        uint64_t id: 16;
        uint64_t value: 1;
    } set_fault;
    struct {
        uint64_t id: 16;
    } return_fault_control;
    struct {
        uint64_t car_state: 8;
        uint64_t precharge_state: 1;
    } main_hb;
    struct {
        uint64_t left_speed: 16;
        uint64_t right_speed: 16;
        uint64_t left_normal: 16;
        uint64_t right_normal: 16;
    } front_wheel_data;
    struct {
        uint64_t daq_command: 64;
    } daq_command_DASHBOARD;
    uint8_t raw_data[8];
} CanParsedData_t;
/* END AUTO MESSAGE STRUCTURE */

// contains most up to date received
// type for each variable matches that defined in JSON
/* BEGIN AUTO CAN DATA STRUCTURE */
typedef struct {
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_driveline;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_torque_vector;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_main_module;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_precharge;
    struct {
        uint16_t idx;
        uint8_t latched;
    } fault_sync_l4_testing;
    struct {
        uint16_t id;
        uint8_t value;
    } set_fault;
    struct {
        uint16_t id;
    } return_fault_control;
    struct {
        car_state_t car_state;
        uint8_t precharge_state;
        uint8_t stale;
        uint32_t last_rx;
    } main_hb;
    struct {
        uint16_t left_speed;
        uint16_t right_speed;
        uint16_t left_normal;
        uint16_t right_normal;
        uint8_t stale;
        uint32_t last_rx;
    } front_wheel_data;
    struct {
        uint64_t daq_command;
    } daq_command_DASHBOARD;
} can_data_t;
/* END AUTO CAN DATA STRUCTURE */

extern can_data_t can_data;

/* BEGIN AUTO EXTERN CALLBACK */
extern void daq_command_DASHBOARD_CALLBACK(CanMsgTypeDef_t* msg_header_a);
extern void fault_sync_driveline_CALLBACK(CanParsedData_t* msg_data_a);
extern void fault_sync_torque_vector_CALLBACK(CanParsedData_t* msg_data_a);
extern void fault_sync_main_module_CALLBACK(CanParsedData_t* msg_data_a);
extern void fault_sync_precharge_CALLBACK(CanParsedData_t* msg_data_a);
extern void fault_sync_l4_testing_CALLBACK(CanParsedData_t* msg_data_a);
extern void set_fault_CALLBACK(CanParsedData_t* msg_data_a);
extern void return_fault_control_CALLBACK(CanParsedData_t* msg_data_a);
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

#endif