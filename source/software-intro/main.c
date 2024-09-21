#include "common/psched/psched.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"
#include <math.h>
#include "stm32l432xx.h"

#include "main.h" //module

//pin inits
GPIOInitConfig_t gpio_config[] = {
    /* INITIALIZE THE LED PIN HERE! */
    GPIO_INIT_OUTPUT(GPIOB, 3, GPIO_OUTPUT_LOW_SPEED), 
    GPIO_INIT_INPUT(GPIOA, 3, GPIO_INPUT_PULL_UP)
    //"TODO: CAN ports."
};

//clock config
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

//private fn headers
void HardFault_Handler(void);
void led_blink(void);
void EXTI3_IRQHandler(void);

int main(void)
{
    //HAL init
    if (0 != PHAL_configureClockRates(&clock_config)){
        HardFault_Handler();
    }
    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t))){
        HardFault_Handler();
    }


    //part 2
    //goal pin A3 (EXTI3 = 000) which is bits 12, 13, 14 on SYSCFG_EXTICR1

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // RCC->APB2ENR |= 1;

    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3; //clear bits 12, 13, 14 (2nd nibble = ?000 on 2nd byte)
    EXTI->FTSR1 |= (1 << 3); //falling edge setup, set last bit for FT0
    EXTI->IMR1 |= (1 << 3); //interrupt mask register 1
    NVIC_EnableIRQ(EXTI3_IRQn); 
    //TODO: init CAN

    /* Initialize the Scheduler */
    schedInit(APB1ClockRateHz);

    /* Task Creation */
    taskCreate(led_blink, 50); 

    //TODO: CAN background tasks
    
    /* Start all tasks */
    schedStart();
    
    return 0;

} 

//called by scheduler task
void led_blink(){
    PHAL_toggleGPIO(GPIOB, 3);
}

void EXTI3_IRQHandler(){
    if(EXTI->PR1 & (1 << 3) ){
        EXTI->PR1 = (1 << 3); //marks pending interrupt 1 as complete for pin 3

        //do something
        
    }
}


//hardfault handler -> effectively pauses scheduler by starting an infinite loop
void HardFault_Handler() 
{
    while(1)
    {
        __asm__("nop");
    }

} 