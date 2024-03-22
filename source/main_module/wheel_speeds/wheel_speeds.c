#include "wheel_speeds.h"

bool wheelSpeedsInit() {
    /* FLOW_RATE_1_TIM */
    // CH1
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    FLOW_RATE_1_TIM->CR1 &= ~TIM_CR1_CEN; // Disable counter (turn off timer)

    FLOW_RATE_1_TIM->PSC = 1 - 1;
    FLOW_RATE_1_TIM->ARR = 0xFFFF - 1;

    /* Set input capture mode */
    FLOW_RATE_1_TIM->CCER &= ~TIM_CCER_CC1E; // Turn off the channel (necessary to write CC1S bits)
    FLOW_RATE_1_TIM->CCER &= ~TIM_CCER_CC2E; // Turn off the channel (necessary to write CC2S bits)

    /* Setup capture compare 1 (period) */
    FLOW_RATE_1_TIM->CCMR1 &= ~TIM_CCMR1_CC1S;
    FLOW_RATE_1_TIM->CCMR1 |= TIM_CCMR1_CC1S_0; // Map IC1 to TI1

    /* Setup capture compare 2 (duty cycle) */
    FLOW_RATE_1_TIM->CCMR1 &= ~TIM_CCMR1_CC2S;
    FLOW_RATE_1_TIM->CCMR1 |= TIM_CCMR1_CC2S_1; // Map IC2 to TI1

    /* CCR1 (period) needs rising edge */
    FLOW_RATE_1_TIM->CCER &= ~TIM_CCER_CC1P;
    FLOW_RATE_1_TIM->CCER &= ~TIM_CCER_CC1NP;

    /* CCR2 (duty cycle) needs falling edge */
    FLOW_RATE_1_TIM->CCER |= TIM_CCER_CC2P;
    FLOW_RATE_1_TIM->CCER &= ~TIM_CCER_CC2NP;

    /* Select trigger */
    FLOW_RATE_1_TIM->SMCR &= ~TIM_SMCR_TS;
    FLOW_RATE_1_TIM->SMCR |= TIM_SMCR_TS_2 | TIM_SMCR_TS_0;

    /* Select trigger */
    FLOW_RATE_1_TIM->SMCR &= ~TIM_SMCR_SMS;
    FLOW_RATE_1_TIM->SMCR |= TIM_SMCR_SMS_2;

    /* Enable channels */
    FLOW_RATE_1_TIM->CCER |= TIM_CCER_CC1E; // Enable CCR1
    FLOW_RATE_1_TIM->CCER |= TIM_CCER_CC2E; // Enable CCR2

    FLOW_RATE_1_TIM->CR1 |= TIM_CR1_CEN; // Enable timer
}
