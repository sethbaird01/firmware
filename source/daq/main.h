#ifndef _MAIN_H_
#define _MAIN_H_

// #define EN_CAN2

typedef enum {
    RX_TAIL_CAN_RX, //!< CAN rx message parsing
    RX_TAIL_SD,     //!< SD Card
    RX_TAIL_UDP,    //!< UDP Broadcast
    RX_TAIL_USB,    //!< USB Send
    RX_TAIL_COUNT,
} rx_tail_t;

typedef enum {
    TCP_RX_TAIL_CAN_TX,
    TCP_RX_TAIL_SD,
    TCP_RX_TAIL_COUNT,
} tcp_rx_tail_t;

// #define DEBUG_LOG
#include "log.h"
#include "common/queue/queue.h"
#include "buffer.h"

#ifdef DEBUG_LOG
// int _write(int handle, char* data, int size);
void _log_str(char* data);
#endif

// #define DISCO_BOARD

// DISCO
#define RED    14
#define BLUE   15
#define GREEN  12
#define ORANGE 13

#define RED_LED_PORT    GPIOD
#define RED_LED_PIN     RED
#define BLUE_LED_PORT   GPIOD
#define BLUE_LED_PIN    BLUE
#define GREEN_LED_PORT  GPIOD
#define GREEN_LED_PIN   GREEN
#define ORANGE_LED_PORT GPIOD
#define ORANGE_LED_PIN  ORANGE
#define USER_BTN_PORT   GPIOA
#define USER_BTN_PIN    0

// Status LEDs
#ifdef DISCO_BOARD
#define HEARTBEAT_LED_PORT   BLUE_LED_PORT
#define HEARTBEAT_LED_PIN    BLUE_LED_PIN
#define CONNECTION_LED_PORT  ORANGE_LED_PORT
#define CONNECTION_LED_PIN   ORANGE_LED_PIN
#define ERROR_LED_PORT       RED_LED_PORT
#define ERROR_LED_PIN        RED_LED_PIN
#define SD_ACTIVITY_LED_PORT GREEN_LED_PORT
#define SD_ACTIVITY_LED_PIN  GREEN_LED_PIN
#define SD_ERROR_LED_PORT    RED_LED_PORT
#define SD_ERROR_LED_PIN     RED_LED_PIN
#else
#define HEARTBEAT_LED_PORT   GPIOA
#define HEARTBEAT_LED_PIN    0
#define CONNECTION_LED_PORT  GPIOA
#define CONNECTION_LED_PIN   1
#define ERROR_LED_PORT       GPIOA
#define ERROR_LED_PIN        4
#define SD_ACTIVITY_LED_PORT GPIOA
#define SD_ACTIVITY_LED_PIN  9
#define SD_ERROR_LED_PORT    GPIOA
#define SD_ERROR_LED_PIN     8
#endif

// TODO: SD Detect LED
#define SD_DETECT_LED_PORT GPIOA
#define SD_DETECT_LED_PIN  10
#define SD_CD_PORT         GPIOC
#define SD_CD_PIN          6

#define ETH_CS_PORT  GPIOB
#define ETH_CS_PIN   12
#define ETH_RST_PORT GPIOB
#define ETH_RST_PIN  11

#define PWR_LOSS_PORT GPIOE
#define PWR_LOSS_PIN  15


#define LOG_ENABLE_PORT GPIOC
#define LOG_ENABLE_PIN  7

#define PER 1
#define GREAT PER

extern volatile uint32_t tick_ms; // Systick 1ms counter
extern q_handle_t q_tx_can;
extern b_handle_t b_rx_can;
extern b_handle_t b_rx_tcp;

extern void HardFault_Handler();

#endif