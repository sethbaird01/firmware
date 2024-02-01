/**
 * @file daq.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Embedded DAQ protocol meant to communicate with a PC dashboard over CAN
 * @version 0.1
 * @date 2021-10-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _DAQ_H_
#define _DAQ_H_

#include "can_parse.h"

// Make this match the node name within the daq_config.json
#define NODE_NAME "Main_Module"

#define DAQ_UPDATE_PERIOD 15 // ms

// BEGIN AUTO VAR COUNT
#define NUM_VARS 20
// END AUTO VAR COUNT

// BEGIN AUTO VAR IDs
#define DAQ_ID_DT_TEMP_ERROR 0
#define DAQ_ID_BAT_TEMP_ERROR 1
#define DAQ_ID_CAL_STEER_ANGLE 2
#define DAQ_ID_COOLING_DAQ_OVERRIDE 3
#define DAQ_ID_SDC_MAIN_STATUS 4
#define DAQ_ID_SDC_CENTER_STOP_STATUS 5
#define DAQ_ID_SDC_INERTIA_SW_STATUS 6
#define DAQ_ID_SDC_BOTS_STAT 7
#define DAQ_ID_SDC_BSPD_STAT 8
#define DAQ_ID_SDC_BMS_STAT 9
#define DAQ_ID_SDC_IMD_STAT 10
#define DAQ_ID_SDC_R_ESTOP_STAT 11
#define DAQ_ID_SDC_L_ESTOP_STAT 12
#define DAQ_ID_SDC_HVD_STAT 13
#define DAQ_ID_SDC_REAR_HUB_STAT 14
#define DAQ_ID_SDC_TSMS_STAT 15
#define DAQ_ID_SDC_PCHG_OUT_STAT 16
#define DAQ_ID_DAQ_BUZZER 17
#define DAQ_ID_DAQ_BRAKE 18
#define DAQ_ID_DAQ_BUZZER_BRAKE_STATUS 19
// END AUTO VAR IDs

// BEGIN AUTO FILE STRUCTS
// END AUTO FILE STRUCTS

bool daqInit(q_handle_t* tx_a);
void daqPeriodic();

#endif