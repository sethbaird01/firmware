/**
 * @file main.c
 * @author Purdue Electric Racing
 * @brief Software Intro Project
 * @version 0.1
 * @date 2024-08-24
 *
 * @copyright Copyright (c) 2024
 *
 */

/* -------------------------------------------------------
    System Includes 
-------------------------------------------------------- */
#include "common/psched/psched.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"
#include <math.h>
#include "stm32l432xx.h"

/* -------------------------------------------------------
    Module Includes 
-------------------------------------------------------- */
#include "main.h"

/* ------------------------------------------------------- 
    Pin Initialization
-------------------------------------------------------- */
GPIOInitConfig_t gpio_config[] =
{
    GPIO_INIT_OUTPUT(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_OUTPUT_LOW_SPEED)
};

/* -------------------------------------------------------
    Clock Configuration
-------------------------------------------------------- */
ClockRateConfig_t clock_config =
{
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .system_clock_target_hz     =80000000,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =160000000,
    .ahb_clock_target_hz        =80000000,
    .apb1_clock_target_hz       =80000000,// / 16,
    .apb2_clock_target_hz       =80000000 / 16,
};

/* -------------------------------------------------------
    Clock Rates
-------------------------------------------------------- */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

/* -------------------------------------------------------
    Procedures
-------------------------------------------------------- */
void ledBlink(void);
void HardFault_Handler(void);

/**
 * Procedure: main()
 * 
 * @brief entry point
 * 
 */
int main(void)
{
    /* HAL Initilization */
    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    /* Initialize the scheduler */
    schedInit(APB1ClockRateHz);

    /* Task Creation */
    taskCreate(ledBlink, 500);
    
    /* Start all tasks */
    schedStart();
    
    return 0;

} /* main() */


/**
 * Procedure: ledBlink()
 * 
 * @brief led blinking function
 * 
 */
void ledBlink()
{
    PHAL_toggleGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    
} /* ledBlink() */


/**
 * Procedure: HardFault_Handler()
 * 
 * @brief Handler for HardFault exceptions.
 * 
 * This function is called when a HardFault exception occurs. It pauses the scheduler
 * and enters an infinite loop
 */
void HardFault_Handler()
{
    PHAL_writeGPIO(LED_GREEN_GPIO_Port, LED_GREEN_Pin, 0);
    while(1)
    {
        __asm__("nop");
    }

} /* HardFault_Handler() */
