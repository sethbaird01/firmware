/**
 * @file main.h
 * @author Luke Oxley (lcoxley@purdue.edu)
 * @brief  Software for controlling vehicle power
 *         distribution and monitoring
 * @version 0.1
 * @date 2023-11-09
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef _LED_H_
#define _LED_H_

#include "common/phal_F4_F7/spi/spi.h"

typedef struct {
    GPIO_TypeDef *blank_port;
    GPIO_TypeDef *latch_port;
    uint8_t blank_pin;
    uint8_t latch_pin;

    SPI_InitConfig_t *spi;
} led_init_t;

extern void initLED(led_init_t *led);
extern void toggleLatch(led_init_t *led);
extern void sendword(led_init_t *led, uint16_t word);

#endif