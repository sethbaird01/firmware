/**
 * @file main.c
 * @author Christopher McGalliard
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
    /* INITIALIZE THE LED PIN HERE! */
    GPIO_INIT_OUTPUT(GPIOB, 3, GPIO_OUTPUT_LOW_SPEED)

    //"TODO: CAN ports."
};

/* -------------------------------------------------------
    Clock Configuration
-------------------------------------------------------- */
ClockRateConfig_t clock_config = 
{
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

/* -------------------------------------------------------
    Clock Rates
-------------------------------------------------------- */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

//TODO: queue definitions

/* -------------------------------------------------------
    Procedures
-------------------------------------------------------- */
//TODO: ledBlink function header
void HardFault_Handler(void);
void led_blink(void);
// void EXTICR0_IRQHandler(void);

/**
 * Procedure: main()
 * 
 * @brief entry point
 * 
 */
int main(void)
{
    //TODO: init queues

    /* HAL Initilization */
    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }
    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    //TODO: init CAN

    //PB[3] pin is 0001
    // SYSCFG_EXTICR1_EXTI0_PB |= (1); //0001: PB[3] pin
    // EXTI->IMR1 |= (1 << 3);
    // NVID_EnableIRQ(EXTI0_IRQn);
    
    /* Initialize the Scheduler */
    schedInit(APB1ClockRateHz);

    /* Task Creation */

    taskCreate(led_blink, 50);    
    //TODO: CAN background tasks
    
    /* Start all tasks */
    schedStart();
    
    return 0;

} /* main() */

void led_blink(){
    PHAL_toggleGPIO(GPIOB, 3);
}

// void EXTICR0_IRQHandler(){

// }

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
    while(1)
    {
        __asm__("nop");
    }

} /* HardFault_Handler() */