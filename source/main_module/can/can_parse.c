/**
 * @file can_parse.c
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief Parsing of CAN messages using auto-generated structures with bit-fields
 * @version 0.1
 * @date 2021-09-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "can_parse.h"

// prototypes
bool initCANFilter();

can_data_t can_data;
q_handle_t* q_rx_can_a;

void initCANParse(q_handle_t* rx_a)
{
    q_rx_can_a = rx_a;
    initCANFilter();
}

uint32_t curr_tick = 0;

void canRxUpdate()
{
    curr_tick += 1;

    CanMsgTypeDef_t msg_header;
    CanParsedData_t* msg_data_a;

    if(qReceive(q_rx_can_a, &msg_header) == SUCCESS_G)
    {
        msg_data_a = (CanParsedData_t *) &msg_header.Data;
        /* BEGIN AUTO CASES */
        switch(msg_header.ExtId)
        {
            case ID_RAW_THROTTLE_BRAKE:
                can_data.raw_throttle_brake.throttle0 = msg_data_a->raw_throttle_brake.throttle0;
                can_data.raw_throttle_brake.throttle1 = msg_data_a->raw_throttle_brake.throttle1;
                can_data.raw_throttle_brake.brake0 = msg_data_a->raw_throttle_brake.brake0;
                can_data.raw_throttle_brake.brake1 = msg_data_a->raw_throttle_brake.brake1;
                can_data.raw_throttle_brake.stale = 0;
                can_data.raw_throttle_brake.last_rx = curr_tick;
                break;
            case ID_START_BUTTON:
                can_data.start_button.start = msg_data_a->start_button.start;
                start_button_CALLBACK(msg_data_a);
                break;
            case ID_PACK_INFO_LV:
                can_data.pack_info_lv.volts = msg_data_a->pack_info_lv.volts;
                can_data.pack_info_lv.error = msg_data_a->pack_info_lv.error;
                can_data.pack_info_lv.bal_flags = msg_data_a->pack_info_lv.bal_flags;
                break;
            case ID_CELL_INFO_LV:
                can_data.cell_info_lv.delta = msg_data_a->cell_info_lv.delta;
                can_data.cell_info_lv.ov = msg_data_a->cell_info_lv.ov;
                can_data.cell_info_lv.uv = msg_data_a->cell_info_lv.uv;
                break;
            default:
                __asm__("nop");
        }
        /* END AUTO CASES */
    }

    /* BEGIN AUTO STALE CHECKS */
    CHECK_STALE(can_data.raw_throttle_brake.stale,
                curr_tick, can_data.raw_throttle_brake.last_rx,
                UP_RAW_THROTTLE_BRAKE);
    /* END AUTO STALE CHECKS */
}

bool initCANFilter()
{
    CAN1->MCR |= CAN_MCR_INRQ;                // Enter back into INIT state (required for changing scale)
    uint32_t timeout = 0;
    while(!(CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;
    if (timeout == PHAL_CAN_INIT_TIMEOUT)
         return false;

    CAN1->FMR  |= CAN_FMR_FINIT;              // Enter init mode for filter banks
    CAN1->FM1R |= 0x07FFFFFF;                 // Set banks 0-27 to id mode
    CAN1->FS1R |= 0x07FFFFFF;                 // Set banks 0-27 to 32-bit scale

    /* BEGIN AUTO FILTER */
    CAN1->FA1R |= (1 << 0);    // configure bank 0
    CAN1->sFilterRegister[0].FR1 = (ID_RAW_THROTTLE_BRAKE << 3) | 4;
    CAN1->sFilterRegister[0].FR2 = (ID_START_BUTTON << 3) | 4;
    CAN1->FA1R |= (1 << 1);    // configure bank 1
    CAN1->sFilterRegister[1].FR1 = (ID_PACK_INFO_LV << 3) | 4;
    CAN1->sFilterRegister[1].FR2 = (ID_CELL_INFO_LV << 3) | 4;
    /* END AUTO FILTER */

    CAN1->FMR  &= ~CAN_FMR_FINIT;             // Enable Filters (exit filter init mode)

    // Enter back into NORMAL mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while((CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;

    return timeout != PHAL_CAN_INIT_TIMEOUT;
}


void canProcessRxIRQs(CanMsgTypeDef_t* rx)
{
    CanParsedData_t* msg_data_a;

    msg_data_a = (CanParsedData_t *) rx->Data;
    switch(rx->ExtId)
    {
        /* BEGIN AUTO RX IRQ */
        /* END AUTO RX IRQ */
        default:
            __asm__("nop");
    }
}
