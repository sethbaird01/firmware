/**
 * @file main.c
 * @author Purdue Electric Racing
 * @brief Dashboard main operation
 * @version 0.1
 * @date 2024-05-28
 *
 * @copyright Copyright (c) 2024
 *
 */

/* -------------------------------------------------------
    System Includes 
-------------------------------------------------------- */
#include "common/bootloader/bootloader_common.h"
#include "common/common_defs/common_defs.h"
#include "common/faults/faults.h"
#include "common/phal_F4_F7/adc/adc.h"
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/psched/psched.h"

/* -------------------------------------------------------
    Module Includes 
-------------------------------------------------------- */
#include "can_parse.h"
#include "daq.h"
#include "lcd.h"
#include "main.h"
#include "nextion.h"
#include "pedals.h"

/* -------------------------------------------------------
    Pin Initialization
-------------------------------------------------------- */
GPIOInitConfig_t gpio_config[] = 
{
/* -------------------------------------------------------
    PORT A
-------------------------------------------------------- */
    GPIO_INIT_ANALOG(BRK_1_GPIO_Port, BRK_1_Pin),                                               /* PA0: BRK1_FLT */
    GPIO_INIT_ANALOG(BRK_2_GPIO_Port, BRK_2_Pin),                                               /* PA1: BRK2_FLT */
    GPIO_INIT_ANALOG(THTL_1_GPIO_Port, THTL_1_Pin),                                             /* PA2: THRTL1_FLT */
    GPIO_INIT_ANALOG(THTL_2_GPIO_Port, THTL_2_Pin),                                             /* PA3: THRTL2_FLT */
    /* PA4: UNUSED */
    /* PA5: UNUSED */
    GPIO_INIT_INPUT(BRK_FAIL_TAP_GPIO_Port, BRK_FAIL_TAP_Pin, GPIO_INPUT_OPEN_DRAIN),           /* PA6: BRK_FAIL_TAP */
    GPIO_INIT_INPUT(BRK_STAT_TAP_GPIO_Port, BRK_STAT_TAP_Pin, GPIO_INPUT_OPEN_DRAIN),           /* PA7: BRK_STAT_TAP */
    /* PA8: UNUSED */
    GPIO_INIT_USART1TX_PA9,                                                                     /* PA9: USART_LCD_TX */
    GPIO_INIT_USART1RX_PA10,                                                                    /* PA10: USART_LCD_RX */
    /* PA11: UNUSED */
    /* PA12: UNUSED */
    /* PA13: SWDIO */
    /* PA14: SWCLK */
    /* PA15: UNUSED */

/* -------------------------------------------------------
    PORT B
-------------------------------------------------------- */
    GPIO_INIT_ANALOG(LOAD_FL_GPIO_Port, LOAD_FL_Pin),                                           /* PB0: LOAD_FR_FLT */
    GPIO_INIT_ANALOG(LOAD_FR_GPIO_Port, LOAD_FR_Pin),                                           /* PB1: LOAD_FL_FLT */
    /* PB2: UNUSED */
    /* PB3: UNUSED */
    /* PB4: UNUSED */
    /* PB5: UNUSED */
    /* PB6: UNUSED */
    /* PB7: UNUSED */
    /* PB8: UNUSED */
    /* PB9: UNUSED */
    /* PB10: UNUSED */
    GPIO_INIT_OUTPUT(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, GPIO_OUTPUT_LOW_SPEED),              /* PB11: SPI2_nWP */
    GPIO_INIT_OUTPUT(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_OUTPUT_LOW_SPEED),              /* PB12: SPI2_NSS_EEPROM */
    GPIO_INIT_SPI2_SCK_PB13,                                                                    /* PB13: SPI2_SCK */
    GPIO_INIT_SPI2_MISO_PB14,                                                                   /* PB14: SPI2_MISO */
    GPIO_INIT_SPI2_MOSI_PB15,                                                                   /* PB15: SPI2_MOSI */

/* -------------------------------------------------------
    PORT C
-------------------------------------------------------- */
    GPIO_INIT_ANALOG(SHOCK_POT_L_GPIO_Port, SHOCK_POT_L_Pin),                                   /* PC0: PotAmpL */
    GPIO_INIT_ANALOG(SHOCK_POT_R_GPIO_Port, SHOCK_POT_R_Pin),                                   /* PC1: PotAmpR */
    GPIO_INIT_ANALOG(LV_5V_V_SENSE_GPIO_Port, LV_5V_V_SENSE_Pin),                               /* PC2: LV-5V-V-Sense */
    GPIO_INIT_ANALOG(LV_3V3_V_SENSE_GPIO_Port, LV_3V3_V_SENSE_Pin),                             /* PC3: LV-3V3-V-Sense */
    GPIO_INIT_ANALOG(LV_12_V_SENSE_GPIO_Port, LV_12_V_SENSE_Pin),                               /* PC4: LV-12V-V-Sense */
    GPIO_INIT_ANALOG(LV_24_V_SENSE_GPIO_Port, LV_24_V_SENSE_Pin),                               /* PC5: LV-24V-V_Sense */
    /* PC6: UNUSED */
    /* PC7: UNUSED */
    GPIO_INIT_INPUT(LV_24_V_FAULT_GPIO_Port, LV_24_V_FAULT_Pin, GPIO_INPUT_OPEN_DRAIN),         /* PC8: NF_NFAULT */
    /* PC9: UNUSED */
    /* PC10: UNUSED */
    /* PC11: UNUSED */
    /* PC12: UNUSED */
    /* PC13: UNUSED */
    /* PC14: UNUSED */
    /* PC15: UNUSED */

/* -------------------------------------------------------
    PORT D
-------------------------------------------------------- */
    GPIO_INIT_CANRX_PD0,                                                                        /* PD0: CAN_RX */
    GPIO_INIT_CANTX_PD1,                                                                        /* PD1: CAN_TX */
    /* PD2: UNUSED */
    /* PD3: UNUSED */
    /* PD4: UNUSED */
    /* PD5: UNUSED */
    /* PD6: UNUSED */
    /* PD7: UNUSED */
    GPIO_INIT_INPUT(DAQ_SWITCH_GPIO_Port, DAQ_SWITCH_Pin, GPIO_INPUT_OPEN_DRAIN),               /* PD8: DAQ_SW_FLT */
    GPIO_INIT_INPUT(ENC_B_GPIO_Port, ENC_B_Pin, GPIO_INPUT_OPEN_DRAIN),                         /* PD9: ENC_B_FLT */
    GPIO_INIT_INPUT(ENC_A_GPIO_Port, ENC_A_Pin, GPIO_INPUT_OPEN_DRAIN),                         /* PD10: ENC_A_FLT */
    GPIO_INIT_INPUT(START_BTN_GPIO_Port, START_BTN_Pin, GPIO_INPUT_PULL_UP),                    /* PD11: START_FLT */
    GPIO_INIT_INPUT(B_DOWN_GPIO_Port, B_DOWN_Pin, GPIO_INPUT_OPEN_DRAIN),                       /* PD12: B3_FLT */
    GPIO_INIT_INPUT(B_OK_GPIO_Port, B_OK_Pin, GPIO_INPUT_OPEN_DRAIN),                           /* PD13: B2_FLT */
    GPIO_INIT_INPUT(B_UP_GPIO_Port, B_UP_Pin, GPIO_INPUT_OPEN_DRAIN),                           /* PD14: B1_FLT */
    /* PD15: UNUSED */

/* -------------------------------------------------------
    PORT E
-------------------------------------------------------- */
    /* PE0: UNUSED */
    GPIO_INIT_OUTPUT_OPEN_DRAIN(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, GPIO_OUTPUT_LOW_SPEED),     /* PE1: PRCHG_LED */
    GPIO_INIT_OUTPUT_OPEN_DRAIN(IMD_LED_GPIO_Port, IMD_LED_Pin, GPIO_OUTPUT_LOW_SPEED),         /* PE2: IMD_LED */
    GPIO_INIT_OUTPUT_OPEN_DRAIN(BMS_LED_GPIO_Port, BMS_LED_Pin, GPIO_OUTPUT_LOW_SPEED),         /* PE3: BMS_LED */
    /* PE4: UNUSED */
    /* PE5: UNUSED */
    /* PE6: UNUSED */
    GPIO_INIT_OUTPUT(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),                /* PE7: ERR_LED */
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),                  /* PE8: CONN_LED */
    GPIO_INIT_OUTPUT(HEART_LED_GPIO_Port, HEART_LED_Pin, GPIO_OUTPUT_LOW_SPEED),                /* PE9: HEARTBEAT */
    /* PE10: UNUSED */
    /* PE11: UNUSED */
    /* PE12: UNUSED */
    /* PE13: UNUSED */
    /* PE14: UNUSED */
    /* PE15: UNUSED */
};

/* -------------------------------------------------------
    ADC 
-------------------------------------------------------- */
volatile raw_adc_values_t raw_adc_values;   /* Struct for all raw ADC channel data */

/* -------------------------------------------------------
    ADC1 Peripheral Config
-------------------------------------------------------- */
ADCInitConfig_t adc_config = 
{
   .clock_prescaler = ADC_CLK_PRESC_6,
   .resolution      = ADC_RES_12_BIT,
   .data_align      = ADC_DATA_ALIGN_RIGHT,
   .cont_conv_mode  = true,
   .dma_mode        = ADC_DMA_CIRCULAR,
   .adc_number      = 1,
};

/* -------------------------------------------------------
    ADC1 Channel Configuration
-------------------------------------------------------- */
ADCChannelConfig_t adc_channel_config[] = 
{
    {.channel=THTL_1_ADC_CHNL,         .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=THTL_2_ADC_CHNL,         .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=BRK_1_ADC_CHNL,          .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=BRK_2_ADC_CHNL,          .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SHOCK_POT_L_ADC_CH,      .rank=5,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SHOCK_POT_R_ADC_CH,      .rank=6,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_5V_V_SENSE_ADC_CHNL,  .rank=7,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_3V3_V_SENSE_ADC_CHNL, .rank=8,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_12_V_SENSE_ADC_CHNL,  .rank=9,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LV_24_V_SENSE_ADC_CHNL,  .rank=10, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LOAD_FL_ADC_CH,          .rank=11, .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LOAD_FR_ADC_CH,          .rank=12, .sampling_time=ADC_CHN_SMP_CYCLES_480},
};

/* -------------------------------------------------------
    ADC1 DMA Configuration
-------------------------------------------------------- */
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &raw_adc_values, sizeof(raw_adc_values) / sizeof(raw_adc_values.t1), 0b01);

/* -------------------------------------------------------
    USART
-------------------------------------------------------- */
q_handle_t q_tx_usart;  /* Global USART1 TX Queue */

/* -------------------------------------------------------
    USART1 DMA Configuration
-------------------------------------------------------- */
dma_init_t usart_tx_dma_config = USART1_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_rx_dma_config = USART1_RXDMA_CONT_CONFIG(NULL, 2);

/* -------------------------------------------------------
    USART1 Configuration (LCD Screen Communication)
-------------------------------------------------------- */
usart_init_t lcd = 
{
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART1,
   .wake_addr   = false,
   .usart_active_num = USART1_ACTIVE_IDX,
   .tx_dma_cfg = &usart_tx_dma_config,
   .rx_dma_cfg = &usart_rx_dma_config
};

/* -------------------------------------------------------
    Clock Configuration
-------------------------------------------------------- */
ClockRateConfig_t clock_config = 
{
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .vco_output_rate_target_hz  =160000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

/* -------------------------------------------------------
    Clock Rates
-------------------------------------------------------- */
extern uint32_t APB1ClockRateHz;        /* Defined in rcc.c */
extern uint32_t APB2ClockRateHz;        /* Defined in rcc.c */
extern uint32_t AHBClockRateHz;         /* Defined in rcc.c */
extern uint32_t PLLClockRateHz;         /* Defined in rcc.c */

/* -------------------------------------------------------
    LCD DATA
-------------------------------------------------------- */
lcd_t lcd_data = 
{
    .encoder_position = 0,
};
extern page_t curr_page;                /* Defined in lcd.c */    
volatile int8_t prev_rot_state = 0;     /* Previous rotary encoder state */
static volatile uint8_t dashboard_input;
                                        /* Status bits for dashboard input */

static volatile uint32_t last_click_time;
/* [prev_state][current_state] = direction (1 = CW, -1 = CCW, 0 = no movement) */
const int8_t encoder_transition_table[ENC_NUM_STATES][ENC_NUM_STATES] = 
{
    { 0, -1,  1,  0},
    { 1,  0,  0, -1},
    {-1,  0,  0,  1},
    { 0,  1, -1,  0}
};

static uint8_t dash_up_button_input_buffer;
static uint8_t dash_down_button_input_buffer;
uint8_t cmd[NXT_STR_SIZE] = {'\0'};     /* Command buffer for USART transmission */

/* -------------------------------------------------------
    Procedures
-------------------------------------------------------- */
extern void HardFault_Handler(void);

void dashboardRotaryEncoderISR(void);
void enableInterrupts(void);
void heartBeatLED(void);
void interpretLoadSensor(void);
void pollDashboardInput(void);
void preflightAnimation(void);
void preflightChecks(void);
void sendBrakeStatus(void);
void sendShockpotData(void);
void usartTxUpdate(void);
float voltToForce(uint16_t load_read);


/**
 * Procedure: main()
 * 
 * @brief entry point
 * 
 */
int main(void)
{
    qConstruct(&q_tx_usart, NXT_STR_SIZE);  /* TX queue for USART1 */

    /* Temporary trim to offset clock drift effects */
    PHAL_trimHSI(HSI_TRIM_DASHBOARD);   

    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }

    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    if (false == PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config) / sizeof(ADCChannelConfig_t)))
    {
        HardFault_Handler();
    }

    if (false == PHAL_initDMA(&adc_dma_config))
    {
        HardFault_Handler();
    }

    PHAL_startTxfer(&adc_dma_config);   /* Enable ADC1 DMA transfers */
    PHAL_startADC(ADC1);                /* Start reading all ADC1 channels */

    initFaultLibrary(FAULT_NODE_NAME, &q_tx_can1_s[0], ID_FAULT_SYNC_DASHBOARD);    /* Initialize fualt library*/

    PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);      /* Light ON IMD */
    PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 1);      /* Light ON BSM */
    PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 1);  /* Light ON PRCHG */

    schedInit(APB1ClockRateHz);                                     /* Initialize the scheduler */
    
    /* Preflight */
    configureAnim(preflightAnimation, preflightChecks, 60, 2500);

    /* Periodic Tasks */
    taskCreate(updateFaultDisplay, 500);
    taskCreate(updateFaultPageIndicators, 500);
    taskCreate(heartBeatLED, 500);
    taskCreate(pedalsPeriodic, 15);
    taskCreate(pollDashboardInput, 25);
    taskCreate(heartBeatTask, 100);
    taskCreate(sendShockpotData, 15);
    taskCreate(interpretLoadSensor, 15);
    taskCreate(update_data_pages, 200);
    taskCreate(sendTVParameters, 4000);
    taskCreate(updateSDCDashboard, 500);

    /* Background Tasks */
    taskCreateBackground(usartTxUpdate);
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);

    /* Start all tasks */
    schedStart();

    return 0;

} /* main() */


/**
 * Procedure: CAN1_RX0_IRQHandler()
 * 
 * @brief Interrupt handler for CAN1 RX0.
 * 
 * This function handles the interrupt request for CAN1 RX0 by calling the 
 * `canParseIRQHandler` function, passing the CAN1 instance as a parameter.
 */
void CAN1_RX0_IRQHandler()
{
    canParseIRQHandler(CAN1);

} /* CAN1_RX0_IRQHandler() */


/**
 * Procedure: dashboard_bl_cmd_CALLBACK()
 * 
 * @brief Callback function for handling dashboard bootloader commands.
 * 
 * This function processes received CAN messages related to the dashboard bootloader commands.
 * If the received command is a reset command (`BLCMD_RST`), it triggers the bootloader reset 
 * for firmware download.
 * 
 * @param msg_data_a Pointer to the parsed CAN message data.
 */
void dashboard_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.dashboard_bl_cmd.cmd == BLCMD_RST)
    {
        Bootloader_ResetForFirmwareDownload();
    }

} /* dashboard_bl_cmd_CALLBACK() */


/**
 * Procedure: dashboardRotaryEncoderISR()
 * 
 * @brief Interrupt Service Routine (ISR) for the encoder.
 * 
 * This function handles the encoder interrupts by reading the encoder states,
 * determining the direction of rotation from a state transition table, and 
 * updating the encoder position accordingly. It also ensures the position 
 * stays within the bounds of the number of LCD pages.
 */
void dashboardRotaryEncoderISR() 
{
    uint8_t raw_enc_a = PHAL_readGPIO(ENC_A_GPIO_Port, ENC_A_Pin);
    uint8_t raw_enc_b = PHAL_readGPIO(ENC_B_GPIO_Port, ENC_B_Pin);
    uint8_t current_state = (raw_enc_b | (raw_enc_a << 1));

    /* Get direction from the state transition table */
    int8_t direction = encoder_transition_table[prev_rot_state][current_state];

    if (direction != 0)
    {
        lcd_data.encoder_position += direction;

        if (lcd_data.encoder_position >= LCD_NUM_PAGES) 
        {
            lcd_data.encoder_position -= LCD_NUM_PAGES;
        } 
        else if (lcd_data.encoder_position < 0) 
        {
            lcd_data.encoder_position += LCD_NUM_PAGES;
        }
    }

    prev_rot_state = current_state;

} /* dashboardRotaryEncoderISR() */


/**
 * Procedure: enableInterrupts()
 * 
 * @brief Enables interrupts for the system.
 * 
 * This function configures and enables various external interrupts (EXTI)
 * for the system, mapping GPIO pins to EXTI lines and configuring the interrupt
 * settings for each EXTI line.
 */
void enableInterrupts()
{
    /* Enable the SYSCFG clock for interrupts */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* START_FLT is on PD11 (EXTI11) */
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PD;   /* Map PD11 to EXTI11 */

    EXTI->IMR |= EXTI_IMR_MR11;                      /* Unmask EXTI11 */
    EXTI->RTSR &= ~EXTI_RTSR_TR11;                   /* Disable the rising edge trigger for START_FLT */
    EXTI->FTSR |= EXTI_FTSR_TR11;                    /* Enable the falling edge trigger for START_FLT */

    /* ENC_B_FLT is on PD9 (EXTI9) */
    /* ENC_A_FLT is on PD10 (EXTI10) */
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PD;    /* Map PD9 to EXTI9 */
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PD;   /* Map PD10 to EXTI10 */

    EXTI->IMR  |= (EXTI_IMR_MR9 | EXTI_IMR_MR10);    /* Unmask EXTI9 and EXTI10 */
    EXTI->RTSR |= (EXTI_RTSR_TR9 | EXTI_RTSR_TR10);  /* Enable the rising edge trigger for both ENC_B_FLT and ENC_A_FLT */
    EXTI->FTSR |= (EXTI_FTSR_TR9 | EXTI_FTSR_TR10);  /* Enable the falling edge trigger for both ENC_B_FLT and ENC_A_FLT */

    /* B3_FLT is on PD12 (EXTI 12) */
    /* B2_FLT is on PD13 (EXTI 13) */
    /* B1_FLT is on PD14 (EXTI 14) */
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PD;   /* Map PD12 to EXTI 12 */
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PD;   /* Map PD13 to EXTI 13 */
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI14_PD;   /* Map PD14 to EXTI 14 */

    EXTI->IMR |= (EXTI_IMR_MR12 | EXTI_IMR_MR13 | EXTI_IMR_MR14);         /* Unmask EXTI12, EXTI13, and EXTI 14 */
    EXTI->RTSR &= ~(EXTI_RTSR_TR12 | EXTI_RTSR_TR13 | EXTI_RTSR_TR14);    /* Disable the rising edge trigger for B3_FLT, B2_FLT, B1_FLT */
    EXTI->FTSR |= (EXTI_FTSR_TR12 | EXTI_FTSR_TR13 | EXTI_FTSR_TR14);     /* Enable the falling edge trigger for B3_FLT, B2_FLT, B1_FLT */

    NVIC_EnableIRQ(EXTI9_5_IRQn);                    /* Enable EXTI9_5 IRQ for ENC_B_FLT */
    NVIC_EnableIRQ(EXTI15_10_IRQn);                  /* Enable EXTI15_10 IRQ: START_FLT, ENC_A_FLT, B3_FLT, B2_FLT, B1_FLT */

} /* enableInterrupts() */


/**
 * Procedure: EXTI9_5_IRQHandler()
 * 
 * @brief Interrupt handler for EXTI lines 9 to 5.
 * 
 * This function handles the interrupt request for EXTI lines 9 to 5. 
 * Specifically, it processes the interrupt for ENC_B_FLT, calls the 
 * encoder ISR, updates the dashboard input, and clears the interrupt 
 * pending bit.
 */
void EXTI9_5_IRQHandler(void) 
{
    /* EXTI9 triggered the interrupt (ENC_B_FLT) */
    if (EXTI->PR & EXTI_PR_PR9) 
    {
        dashboardRotaryEncoderISR();
        dashboard_input |= (1 << DASH_INPUT_ROT_ENC);
        EXTI->PR |= EXTI_PR_PR9;        /* Clear the interrupt pending bit for EXTI9 */
    }

} /* EXTI9_5_IRQHandler() */


/**
 * Procedure: EXTI15_10_IRQHandler()
 * 
 * @brief Interrupt handler for EXTI lines 15 to 10.
 * 
 * This function handles the interrupt request for EXTI lines 15 to 10.
 * It processes interrupts for ENC_A_FLT, B1_FLT, B2_FLT, B3_FLT, and 
 * START_FLT, updating the dashboard input and clearing the interrupt 
 * pending bits accordingly.
 */
void EXTI15_10_IRQHandler() 
{
    /* EXTI10 triggered the interrupt (ENC_A_FLT) */
    if (EXTI->PR & EXTI_PR_PR10) 
    {
        dashboardRotaryEncoderISR();
        dashboard_input |= (1 << DASH_INPUT_ROT_ENC);
        EXTI->PR |= EXTI_PR_PR10;       /* Clear the interrupt pending bit for EXTI14 */
    }

    /* EXTI14 triggered the interrupt (B1_FLT) */
    /* This is the TOP button on the dashboard */
    if (EXTI->PR & EXTI_PR_PR14) 
    {
        if (sched.os_ticks - last_click_time < 200) 
        {
            last_click_time = sched.os_ticks;
            EXTI->PR |= EXTI_PR_PR14;       /* Clear the interrupt pending bit for EXTI14 */
        }
        else 
        {
            last_click_time = sched.os_ticks;
            dashboard_input |= (1 << DASH_INPUT_UP_BUTTON);
            EXTI->PR |= EXTI_PR_PR14;       /* Clear the interrupt pending bit for EXTI14 */
        }
    }

    /* EXTI13 triggered the interrupt (B2_FLT) */
    /* This is the MIDDLE button on the dashbaord */
    if (EXTI->PR & EXTI_PR_PR13)
    {
        if (sched.os_ticks - last_click_time < 200) 
        {
            last_click_time = sched.os_ticks;
            EXTI->PR |= EXTI_PR_PR13;       /* Clear the interrupt pending bit for EXTI13 */
        }
        else
        {
            last_click_time = sched.os_ticks;
            dashboard_input |= (1 << DASH_INPUT_DOWN_BUTTON);
            EXTI->PR |= EXTI_PR_PR13;       /* Clear the interrupt pending bit for EXTI13 */
        }
    }

    /* EXTI12 triggered the interrupt (B3_FLT) */
    /* This is the BOTTOM button on the dashboard */
    if (EXTI->PR & EXTI_PR_PR12)
    {
        if (sched.os_ticks - last_click_time < 300) 
        {
            last_click_time = sched.os_ticks;
            EXTI->PR |= EXTI_PR_PR12;       /* Clear the interrupt pending bit for EXTI12 */
        }
        else
        {
            last_click_time = sched.os_ticks;
            dashboard_input |= (1 << DASH_INPUT_SELECT_BUTTON);
            EXTI->PR |= EXTI_PR_PR12;       /* Clear the interrupt pending bit for EXTI12 */
        }
    }

    /* EXTI11 triggered the interrupt (START_FLT) */
    if (EXTI->PR & EXTI_PR_PR11) 
    {
        PHAL_toggleGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin); /* Toggle LED for testing */
        dashboard_input |= (1 << DASH_INPUT_START_BUTTON);
        EXTI->PR |= EXTI_PR_PR11;       /* Clear the interrupt pending bit for EXTI11 */
    }

} /* EXTI15_10_IRQHandler() */


/**
 * Procedure: HardFault_Handler()
 * 
 * @brief Handler for HardFault exceptions.
 * 
 * This function is called when a HardFault exception occurs. It pauses the scheduler
 * and enters an infinite loop where the Independent Watchdog (IWDG) key register is 
 * repeatedly refreshed to prevent a system reset.
 */
void HardFault_Handler()
{
   schedPause();
   while(1)
    {
        IWDG->KR = 0xAAAA;
    }

} /* HardFault_Handler() */


/**
 * Procedure: heartBeatLED()
 * 
 * @brief Handles the heartbeat LED and other status LEDs based on system state.
 * 
 * This function toggles the heartbeat LED, updates the connection LED based on 
 * the time since the last CAN message was received, and updates the precharge, 
 * IMD, and BMS LEDs based on the state of the system. It also sends CAN statistics 
 * every other call.
 */
void heartBeatLED()
{
    static uint8_t imd_prev_latched;
    static uint8_t bms_prev_latched;
    
    PHAL_toggleGPIO(HEART_LED_GPIO_Port, HEART_LED_Pin);
    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    }
    else 
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
    }
    if (!can_data.main_hb.stale && can_data.main_hb.precharge_state) 
    {
        PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 0);
    }
    else 
    {
        PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 1);
    }
    if (!can_data.precharge_hb.stale) 
    {
        if (can_data.precharge_hb.IMD)
        {
            imd_prev_latched = 1;
        }
        if (can_data.precharge_hb.BMS)
        {
            bms_prev_latched = 1;
        }
    }
    else 
    {
        PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
        PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 0);
    }
    
    PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, !imd_prev_latched);
    PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, !bms_prev_latched);

    static uint8_t trig;
    if (trig) 
    {
        SEND_DASH_CAN_STATS(can_stats.tx_of, can_stats.tx_fail, can_stats.rx_of, can_stats.rx_overrun);
    }
    trig = !trig;

} /* heartBeatLED() */


/**
 * Procedure: interpretLoadSensor()
 * 
 * @brief Interprets load sensor values and sends CAN messages with the force information.
 * 
 * This function converts the raw ADC values from the load sensors to force values using 
 * the `voltToForce` function. It then sends a CAN message with the minimal force information 
 * every 15 milliseconds.
 */
void interpretLoadSensor(void) 
{
    float force_load_l = voltToForce(raw_adc_values.load_l);
    float force_load_r = voltToForce(raw_adc_values.load_r);
    SEND_LOAD_SENSOR_READINGS_DASH(force_load_l, force_load_r);

} /* interpretLoadSensor() */


/**
 * Procedure: pollDashboardInput()
 * 
 * @brief Polls for dashboard user input and handles corresponding actions.
 * 
 * This function checks for user input from various dashboard controls, including 
 * encoder inputs, the start button, and the select button. It updates buffers and 
 * states based on the input, and triggers appropriate actions such as moving up 
 * or down in the menu, updating the page, reporting the start button press, or 
 * selecting an item.
 */
void pollDashboardInput()
{
    /* Debounce Up Button */
    dash_up_button_input_buffer <<= 1;
    if (PHAL_readGPIO(GPIOD, 14) == 0)
    {
        dash_up_button_input_buffer |= 1;
    }
    dash_up_button_input_buffer &= 0b00011111;
    if (dash_up_button_input_buffer == 0b00000001)
    {
        moveUp();
    }

    /* Debounce Down Button */
    dash_down_button_input_buffer <<= 1;
    if (PHAL_readGPIO(GPIOD, 13) == 0)
    {
        dash_down_button_input_buffer |= 1;
    }
    dash_down_button_input_buffer &= 0b00011111;
    if (dash_down_button_input_buffer == 0b00000001)
    {
        moveDown();
    }

    if (dashboard_input & (1U << DASH_INPUT_ROT_ENC))
    {
        updatePage();
        dashboard_input &= ~(1U << DASH_INPUT_ROT_ENC);
    }

    /* Check for Start Button Pressed */
    if (dashboard_input & (1U << DASH_INPUT_START_BUTTON))
    {
        SEND_START_BUTTON(1);
        dashboard_input &= ~(1U << DASH_INPUT_START_BUTTON);
    }

    /* Check Select Item Pressed */
    if (dashboard_input & (1U << DASH_INPUT_SELECT_BUTTON))
    {
        selectItem();
        dashboard_input &= ~(1U << DASH_INPUT_SELECT_BUTTON);
    }

} /* pollDashboardInput() */


/**
 * Procedure: preflightAnimation()
 * 
 * @brief Executes the preflight animation sequence.
 * 
 * This function controls the LEDs to perform a preflight animation. It turns on 
 * the BMS, IMD, and precharge LEDs and cycles the heart, connection, and error 
 * LEDs in a specific pattern. Additionally, it turns off the BMS, IMD, and precharge 
 * LEDs periodically.
 */
void preflightAnimation(void) 
{
    static uint32_t time_ext;
    static uint32_t time;
    
    PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 1);
    PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 1);
    PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 1);

    PHAL_writeGPIO(HEART_LED_GPIO_Port, HEART_LED_Pin, 0);
    PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 6)
    {
        case 0:
        case 5:
            PHAL_writeGPIO(HEART_LED_GPIO_Port, HEART_LED_Pin, 1);
            break;
        case 1:
        case 4:
            PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
            break;
        case 2:
        case 3:
            PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
            break;
    }

    switch (time_ext++ % 4)
    {
        case 0:
            PHAL_writeGPIO(BMS_LED_GPIO_Port, BMS_LED_Pin, 0);
            PHAL_writeGPIO(IMD_LED_GPIO_Port, IMD_LED_Pin, 0);
            PHAL_writeGPIO(PRCHG_LED_GPIO_Port, PRCHG_LED_Pin, 0);
            break;
    }

} /* preflightAnimation() */


/**
 * Procedure: preflightChecks()
 * 
 * @brief Performs preflight checks and initialization for various modules.
 * 
 * This function performs a series of preflight checks and initializations for 
 * CAN, USART, ADC, DMA, and other modules. It also enables interrupts, 
 * initializes the LCD, zeros the encoder, and registers the preflight completion.
 */
void preflightChecks(void) 
{
    static uint8_t state;

    switch (state++)
    {
        case 0:
            if (false == PHAL_initCAN(CAN1, false, VCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
            break;
        case 1:
            if (false == PHAL_initUSART(&lcd, APB2ClockRateHz))
            {
                HardFault_Handler();
            }
            break;
        case 2:
            if (false == PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config)/sizeof(ADCChannelConfig_t)))
            {
                HardFault_Handler();
            }
            if (false == PHAL_initDMA(&adc_dma_config))
            {
                HardFault_Handler();
            }
            PHAL_startTxfer(&adc_dma_config);
            PHAL_startADC(ADC1);
            break;
        case 3:
            initCANParse();
            if (daqInit(&q_tx_can1_s[2]))
            {
                HardFault_Handler();
            }
            break;
        case 4:
            enableInterrupts();
            break;
        case 5:
            initLCD();
            break;
        case 6:
            zeroEncoder(&prev_rot_state);
            break;
        default:
            registerPreflightComplete(1);
            state = 255; /* prevent wrap around */
    }

} /* preflightChecks() */


/**
 * Procedure: sendShockpotData()
 * 
 * @brief Sends shock potentiometer values after processing.
 * 
 * This function retrieves raw ADC values for the left and right shock potentiometers, 
 * processes them to scale linearly from 0 to 3744, and then sends the parsed values 
 * to the front shocks. The processed values are calculated considering the maximum 
 * potentiometer distance and voltage ranges, as well as droop values.
 */
void sendShockpotData()
{
    uint16_t shock_l = raw_adc_values.shock_left;
    uint16_t shock_r = raw_adc_values.shock_right;
    int16_t shock_l_parsed;
    int16_t shock_r_parsed;
    /* Will scale linearly from 0 - 3744. so 75 - (percent of 3744 * 75) */
    shock_l_parsed =  -1 * ((POT_MAX_DIST - (int16_t)((shock_l / (POT_VOLT_MIN_L - POT_VOLT_MAX_L)) * POT_MAX_DIST)) - POT_DIST_DROOP_L);
    shock_r_parsed = -1 * ((POT_MAX_DIST - (int16_t)((shock_r / (POT_VOLT_MIN_R - POT_VOLT_MAX_R)) * POT_MAX_DIST)) - POT_DIST_DROOP_R);
    SEND_SHOCK_FRONT(shock_l_parsed, shock_r_parsed);

} /* sendShockpotData() */


/**
 * Procedure: usartTxUpdate()
 * 
 * @brief Updates the USART transmission for the LCD.
 * 
 * This function checks if the USART transmission for the LCD is not busy and if there
 * is a command to send from the transmission queue. If both conditions are met, it 
 * initiates a DMA transmission of the command.
 */
void usartTxUpdate()
{
    if ((false == PHAL_usartTxBusy(&lcd)) && (SUCCESS_G == qReceive(&q_tx_usart, cmd)))
    {
        PHAL_usartTxDma(&lcd, (uint16_t *) cmd, strlen(cmd));
    }
    
} /* usartTxUpdate() */


/**
 * Procedure: voltToForce()
 * 
 * @brief Converts a raw ADC load sensor reading to force in newtons.
 * 
 * This function takes a raw ADC value from the load sensor and converts it 
 * to a force value in newtons. The conversion process involves the following steps:
 * 1. Scale the ADC value to a voltage.
 *    - ADC values range from 0 to 4095 for a 12-bit ADC.
 *    - The reference voltage is 3.3V.
 *    - Voltage (V_out) is calculated as: V_out = (ADC_value / 4095.0) * 3.3
 * 2. Adjust the scaled voltage for the sensor's voltage divider.
 *    - The sensor uses a voltage divider with resistors R1 and R2.
 *    - R1 = 3.4kΩ and R2 = 6.6kΩ.
 *    - The input voltage (V_in) is calculated as: V_in = V_out * (1.0 + (R1 / R2))
 * 3. Convert the adjusted voltage to mass.
 *    - The mass is proportional to the voltage: mass = V_in * 100
 * 4. Convert the mass to force using the gravitational constant.
 *    - Force (F) is calculated as: F = mass * g, where g = 9.8 m/s²
 * 
 * @param load_read Raw ADC value from the load sensor.
 * @return Force in newtons.
 */
float voltToForce(uint16_t load_read) 
{
    return ((load_read / 4095.0 * 3.3) * (1.0 + (3.4 / 6.6))) * 100.0 * 9.8;

} /* voltToForce() */
