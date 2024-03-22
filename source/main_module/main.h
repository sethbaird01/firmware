/**
 * @file main.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Software for controlling SDC based on
 *         faults and cooling based on temperatures
 * @version 0.1
 * @date 2022-03-16
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _MAIN_H_
#define _MAIN_H_


//STM32L496VGT6

#include <stdint.h>
#include "common/faults/fault_nodes.h"

#define FAULT_NODE_NAME NODE_MAIN_MODULE

// Internal Status Indicators
#define ERR_LED_GPIO_Port           (GPIOD)
#define ERR_LED_Pin                 (4)
#define CONN_LED_GPIO_Port          (GPIOD)
#define CONN_LED_Pin                (3)
#define CONN_LED_MS_THRESH          (500)
#define HEARTBEAT_GPIO_Port         (GPIOD)
#define HEARTBEAT_Pin               (2)

// External Status Indicators
#define BRK_LIGHT_GPIO_Port         (GPIOD)
#define BRK_LIGHT_Pin               (14)
#define BUZZER_GPIO_Port            (GPIOD)
#define BUZZER_Pin                  (12)

#define BRK_BUZZER_STAT_GPIO_Port   (GPIOB)
#define BRK_BUZZER_STAT_Pin         (15)

#define TSAL_LVAL_STAT_GPIO_Port    (GPIOD)
#define TSAL_LVAL_STAT_Pin          (10)

// CAN
#define VCAN_RX_GPIO_Port           (GPIOA)
#define VCAN_RX_Pin                 (11)
#define VCAN_TX_GPIO_Port           (GPIOA)
#define VCAN_TX_Pin                 (12)

#define EMCAN_RX_GPIO_Port         (GPIOB)
#define EMCAN_RX_Pin               (12)
#define EMCAN_TX_GPIO_Port         (GPIOB)
#define EMCAN_TX_Pin               (13)

// SPI Peripherals
#define SPI1_SCK_GPIO_Port          (GPIOA)
#define SPI1_SCK_Pin                (5)
#define SPI1_MISO_GPIO_Port         (GPIOA)
#define SPI1_MISO_Pin               (6)
#define SPI1_MOSI_GPIO_Port         (GPIOA)
#define SPI1_MOSI_Pin               (7)

#define EEPROM_nWP_GPIO_Port        (GPIOB)
#define EEPROM_nWP_Pin              (1)
#define EEPROM_NSS_GPIO_Port        (GPIOB)
#define EEPROM_NSS_Pin              (0)

// Shutdown Circuit
#define SDC_CTRL_GPIO_Port          (GPIOD)
#define SDC_CTRL_Pin                (11)

#define SDC_MUX_S0_GPIO_Port        (GPIOC)
#define SDC_MUX_S0_Pin              (9)
#define SDC_MUX_S1_GPIO_Port        (GPIOD)
#define SDC_MUX_S1_Pin              (15)
#define SDC_MUX_S2_GPIO_Port        (GPIOC)
#define SDC_MUX_S2_Pin              (6)
#define SDC_MUX_S3_GPIO_Port        (GPIOC)
#define SDC_MUX_S3_Pin              (7)

#define SDC_MUX_DATA_GPIO_Port      (GPIOC)
#define SDC_MUX_DATA_Pin            (8)


// HV Bus Information
#define V_MC_SENSE_GPIO_Port        (GPIOA)
#define V_MC_SENSE_Pin              (1)
#define V_MC_SENSE_ADC_CHNL         (1)
#define V_BAT_SENSE_GPIO_Port       (GPIOA)
#define V_BAT_SENSE_Pin             (0)
#define V_BAT_SENSE_ADC_CHNL        (0)

#define BMS_STAT_GPIO_Port          (GPIOD)
#define BMS_STAT_Pin                (13)

#define PRCHG_STAT_GPIO_Port        (GPIOE)
#define PRCHG_STAT_Pin              (5)

// Motor Controllers
#define MC_L_INVERT                 (0)
#define MC_L_UART                   (USART1)
#define MC_L_UART_TX_GPIO_Port      (GPIOA)
#define MC_L_UART_TX_GPIO_Pin       (9)
#define MC_L_UART_RX_GPIO_Port      (GPIOA)
#define MC_L_UART_RX_GPIO_Pin       (10)

#define MC_R_INVERT                 (0)
#define MC_R_UART                   (USART2)
#define MC_R_UART_TX_GPIO_Port      (GPIOA)
#define MC_R_UART_TX_GPIO_Pin       (2)
#define MC_R_UART_RX_GPIO_Port      (GPIOA)
#define MC_R_UART_RX_GPIO_Pin       (3)

// Wheel Speed
#define MOTOR_R_WS_PWM_TIM          (TIM1)
#define MOTOR_R_WS_PWM_CH           (1)
#define MOTOR_R_WS_GPIO_Port        (GPIOA)
#define MOTOR_R_WS_Pin              (8)
#define MOTOR_R_WS_AF               (1)

#define MOTOR_L_WS_PWM_TIM          (TIM4)
#define MOTOR_L_WS_PWM_CH           (1)
#define MOTOR_L_WS_GPIO_Port        (GPIOB)
#define MOTOR_L_WS_Pin              (6)
#define MOTOR_L_WS_AF               (2)

// Shock Pots
#define SHOCK_POT_L_GPIO_Port       (GPIOC)
#define SHOCK_POT_L_Pin             (3)
#define SHOCK_POT_L_ADC_CHNL        (13)

#define SHOCK_POT_R_GPIO_Port       (GPIOC)
#define SHOCK_POT_R_Pin             (2)
#define SHOCK_POT_R_ADC_CHNL        (12)

// Load Sensors
#define LOAD_R_GPIO_Port            (GPIOA)
#define LOAD_R_Pin                  (4)
#define LOAD_R_ADC_CHNL             (4)

#define LOAD_L_GPIO_Port            (GPIOC)
#define LOAD_L_Pin                  (1)
#define LOAD_L_ADC_CHNL             (11)

// MCU Internal Thermistor
#define INTERNAL_THERM_ADC_CHNL     (16)

// Thermistor Analog Multiplexer
#define THERM_MUX_S0_GPIO_Port      (GPIOE)
#define THERM_MUX_S0_Pin            (2)
#define THERM_MUX_S1_GPIO_Port      (GPIOE)
#define THERM_MUX_S1_Pin            (3)
#define THERM_MUX_S2_GPIO_Port      (GPIOE)
#define THERM_MUX_S2_Pin            (4)
#define THERM_MUX_D_GPIO_Port       (GPIOC)
#define THERM_MUX_D_Pin             (0)
#define THERM_MUX_D_ADC_CHNL        (10)

#define THERM_MUX_BAT_IN            (0)
#define THERM_MUX_BAT_OUT           (1)
#define THERM_MUX_DT_IN             (2)
#define THERM_MUX_DT_OUT            (3)

#define ADC_REF_mV (3300UL) // mV
#define ADC_REF_fp 3.3F
#define ADC_MAX    4095
typedef struct
{
    // Do not modify this struct unless
    // you modify the ADC DMA config
    // in main.c to match
    uint16_t v_mc;
    uint16_t v_bat;
    uint16_t shock_l;
    uint16_t shock_r;
    uint16_t therm_mux_d;
    uint16_t load_l;
    uint16_t load_r;
    uint16_t therm_mcu;
}__attribute__((packed)) ADCReadings_t;
volatile extern ADCReadings_t adc_readings;

#endif
