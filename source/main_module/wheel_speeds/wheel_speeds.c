#include "wheel_speeds.h"

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;

bool wheelSpeedsInit() {
    /* Right Init */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    MOTOR_R_WS_PWM_TIM->CR1 &= ~TIM_CR1_CEN; // Disable counter (turn off timer)

    MOTOR_R_WS_PWM_TIM->PSC = 1 - 1;
    MOTOR_R_WS_PWM_TIM->ARR = 0xFFFF - 1;

    /* Set input capture mode */
    MOTOR_R_WS_PWM_TIM->CCER &= ~TIM_CCER_CC1E; // Turn off the channel (necessary to write CC1S bits)
    MOTOR_R_WS_PWM_TIM->CCER &= ~TIM_CCER_CC2E; // Turn off the channel (necessary to write CC2S bits)

    /* Setup capture compare 1 (period) */
    MOTOR_R_WS_PWM_TIM->CCMR1 &= ~TIM_CCMR1_CC1S;
    MOTOR_R_WS_PWM_TIM->CCMR1 |= TIM_CCMR1_CC1S_0; // Map IC1 to TI1

    /* Setup capture compare 2 (duty cycle) */
    MOTOR_R_WS_PWM_TIM->CCMR1 &= ~TIM_CCMR1_CC2S;
    MOTOR_R_WS_PWM_TIM->CCMR1 |= TIM_CCMR1_CC2S_1; // Map IC2 to TI1

    /* CCR1 (period) needs rising edge */
    MOTOR_R_WS_PWM_TIM->CCER &= ~TIM_CCER_CC1P;
    MOTOR_R_WS_PWM_TIM->CCER &= ~TIM_CCER_CC1NP;

    /* CCR2 (duty cycle) needs falling edge */
    MOTOR_R_WS_PWM_TIM->CCER |= TIM_CCER_CC2P;
    MOTOR_R_WS_PWM_TIM->CCER &= ~TIM_CCER_CC2NP;

    /* Select trigger */
    MOTOR_R_WS_PWM_TIM->SMCR &= ~TIM_SMCR_TS;
    MOTOR_R_WS_PWM_TIM->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0;

    /* Select trigger */
    MOTOR_R_WS_PWM_TIM->SMCR &= ~TIM_SMCR_SMS;
    MOTOR_R_WS_PWM_TIM->SMCR |= TIM_SMCR_SMS_2;

    /* Enable channels */
    MOTOR_R_WS_PWM_TIM->CCER |= TIM_CCER_CC1E; // Enable CCR1
    MOTOR_R_WS_PWM_TIM->CCER |= TIM_CCER_CC2E; // Enable CCR2

    MOTOR_R_WS_PWM_TIM->CR1 |= TIM_CR1_CEN; // Enable timer

    /* Left Init */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    MOTOR_L_WS_PWM_TIM->CR1 &= ~TIM_CR1_CEN; // Disable counter (turn off timer)

    MOTOR_L_WS_PWM_TIM->PSC = 1 - 1;
    MOTOR_L_WS_PWM_TIM->ARR = 0xFFFF - 1;

    /* Set input capture mode */
    MOTOR_L_WS_PWM_TIM->CCER &= ~TIM_CCER_CC1E; // Turn off the channel (necessary to write CC1S bits)
    MOTOR_L_WS_PWM_TIM->CCER &= ~TIM_CCER_CC2E; // Turn off the channel (necessary to write CC2S bits)

    /* Setup capture compare 1 (period) */
    MOTOR_L_WS_PWM_TIM->CCMR1 &= ~TIM_CCMR1_CC1S;
    MOTOR_L_WS_PWM_TIM->CCMR1 |= TIM_CCMR1_CC1S_0; // Map IC1 to TI1

    /* Setup capture compare 2 (duty cycle) */
    MOTOR_L_WS_PWM_TIM->CCMR1 &= ~TIM_CCMR1_CC2S;
    MOTOR_L_WS_PWM_TIM->CCMR1 |= TIM_CCMR1_CC2S_1; // Map IC2 to TI1

    /* CCR1 (period) needs rising edge */
    MOTOR_L_WS_PWM_TIM->CCER &= ~TIM_CCER_CC1P;
    MOTOR_L_WS_PWM_TIM->CCER &= ~TIM_CCER_CC1NP;

    /* CCR2 (duty cycle) needs falling edge */
    MOTOR_L_WS_PWM_TIM->CCER |= TIM_CCER_CC2P;
    MOTOR_L_WS_PWM_TIM->CCER &= ~TIM_CCER_CC2NP;

    /* Select trigger */
    MOTOR_L_WS_PWM_TIM->SMCR &= ~TIM_SMCR_TS;
    MOTOR_L_WS_PWM_TIM->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0;

    /* Select trigger */
    MOTOR_L_WS_PWM_TIM->SMCR &= ~TIM_SMCR_SMS;
    MOTOR_L_WS_PWM_TIM->SMCR |= TIM_SMCR_SMS_2;

    /* Enable channels */
    MOTOR_L_WS_PWM_TIM->CCER |= TIM_CCER_CC1E; // Enable CCR1
    MOTOR_L_WS_PWM_TIM->CCER |= TIM_CCER_CC2E; // Enable CCR2

    MOTOR_L_WS_PWM_TIM->CR1 |= TIM_CR1_CEN; // Enable timer
    
    return 0;
}

uint32_t getLeftWheelSpeed() {
    /* So we get the period and that will be the period in which it is detecting 
     * each individual metal piece on the wheel/brake thing. We know there are 42 teeth
     * per revolution. So if we scale up the period by 42, that is how often a revolution
     * is occuring. */

    uint32_t speed;
    uint32_t period = (MOTOR_L_WS_PWM_TIM->CCR2 * MOTOR_L_WS_PWM_TIM->PSC + 1.0) / APB2ClockRateHz;
    uint32_t time_per_revolution = period * TEETH_PER_REVOLUTION;

    return speed;
}
