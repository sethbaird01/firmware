/**
 * @file main.c
 * @author Purdue Electric Racing
 * @brief Accumulator Box
 * @version 0.1
 * @date 2024-06-20
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
#include "common/psched/psched.h"
#include "stm32f407xx.h"

/* -------------------------------------------------------
    Module Includes 
-------------------------------------------------------- */
#include "can_parse.h"
#include "daq.h"
#include "main.h"
#include "orion.h"
#include "tmu.h"

/* -------------------------------------------------------
    Pin Initialization
-------------------------------------------------------- */
GPIOInitConfig_t gpio_config[] = 
{
   /* I-Sense */
   GPIO_INIT_ANALOG(I_SENSE_CH1_GPIO_Port, I_SENSE_CH1_Pin),
   GPIO_INIT_ANALOG(I_SENSE_CH2_GPIO_Port, I_SENSE_CH2_Pin),

   /* CAN */
   GPIO_INIT_CANRX_PA11,
   GPIO_INIT_CANTX_PA12,

   /* Status and HV Monitoring */
   GPIO_INIT_OUTPUT_OPEN_DRAIN(BMS_STATUS_GPIO_Port, BMS_STATUS_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_INPUT(IMD_HS_PWM_GPIO_Port, IMD_HS_PWM_Pin, GPIO_INPUT_OPEN_DRAIN),
   GPIO_INIT_INPUT(IMD_LS_PWM_GPIO_Port, IMD_LS_PWM_Pin, GPIO_INPUT_OPEN_DRAIN),
   GPIO_INIT_INPUT(IMD_STATUS_GPIO_Port, IMD_STATUS_Pin, GPIO_INPUT_OPEN_DRAIN),

   /* LEDs */
   GPIO_INIT_OUTPUT(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_OUTPUT(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, GPIO_OUTPUT_LOW_SPEED),

   /* TMU Mux Selects and Measurements */
   GPIO_INIT_OUTPUT(MUX_A_Port, MUX_A_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_OUTPUT(MUX_B_Port, MUX_B_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_OUTPUT(MUX_C_Port, MUX_C_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_OUTPUT(MUX_D_Port, MUX_D_Pin, GPIO_OUTPUT_LOW_SPEED),
   GPIO_INIT_ANALOG(TMU_1_Port, TMU_1_Pin),
   GPIO_INIT_ANALOG(TMU_2_Port, TMU_2_Pin),
   GPIO_INIT_ANALOG(TMU_3_Port, TMU_3_Pin),
   GPIO_INIT_ANALOG(TMU_4_Port, TMU_4_Pin),

   /* Board Temperature Measurement */
   GPIO_INIT_ANALOG(BOARD_TEMP_Port, BOARD_TEMP_Pin),

   /* 5V Rail Monitoring */
   GPIO_INIT_ANALOG(VSENSE_5V_Port, VSENSE_5V_Pin),

   /* Orion BMS Communication (external pull up on PCB) */
   GPIO_INIT_INPUT(BMS_DISCHARGE_ENABLE_Port, BMS_DISCHARGE_ENABLE_Pin, GPIO_INPUT_OPEN_DRAIN),
   GPIO_INIT_INPUT(BMS_CHARGE_ENABLE_Port, BMS_CHARGE_ENABLE_Pin, GPIO_INPUT_OPEN_DRAIN),
   GPIO_INIT_INPUT(BMS_CHARGER_SAFETY_Port, BMS_CHARGER_SAFETY_Pin, GPIO_INPUT_OPEN_DRAIN)
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
extern uint32_t APB1ClockRateHz;        /* Defined in rcc.c */
extern uint32_t APB2ClockRateHz;        /* Defined in rcc.c */
extern uint32_t AHBClockRateHz;         /* Defined in rcc.c */
extern uint32_t PLLClockRateHz;         /* Defined in rcc.c */

/* -------------------------------------------------------
    BMS
-------------------------------------------------------- */
extern uint8_t orion_error;             /* Defined in orion.c                           */
bool bms_daq_override = false;          /* DAQ Variable: Status of DAQ BMS override     */
bool bms_daq_stat = false;              /* DAQ Variable: Status of the BMS through DAQ  */
tmu_handle_t tmu;                       /* Temperature measurement unit main struct     */

/* -------------------------------------------------------
    ADC 
-------------------------------------------------------- */
volatile ADCReadings_t adc_readings;    /* Struct for all raw ADC channel data */

/* -------------------------------------------------------
    ADC1 Peripheral Config
    ADC clock to be 30MHz (upper bound), 
    clocked from APB2 (160/6=27MHz) 
-------------------------------------------------------- */
ADCInitConfig_t adc_config =
{
    .clock_prescaler = ADC_CLK_PRESC_6,
    .resolution      = ADC_RES_12_BIT,
    .data_align      = ADC_DATA_ALIGN_RIGHT,
    .cont_conv_mode  = true,
    .adc_number      = 1,
    .dma_mode        = ADC_DMA_CIRCULAR
};

/* -------------------------------------------------------
    ADC1 Channel Configuration
-------------------------------------------------------- */
ADCChannelConfig_t adc_channel_config[] =
{
    {.channel=TMU_1_ADC_CHANNEL,        .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=TMU_2_ADC_CHANNEL,        .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=TMU_3_ADC_CHANNEL,        .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=TMU_4_ADC_CHANNEL,        .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=I_SENSE_CH1_ADC_CHANNEL,  .rank=5,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=I_SENSE_CH2_ADC_CHANNEL,  .rank=6,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
};

/* -------------------------------------------------------
    ADC1 DMA Configuration
-------------------------------------------------------- */
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &adc_readings,
            sizeof(adc_readings) / sizeof(adc_readings.tmu_1), 0b01);

/* -------------------------------------------------------
    Procedures
-------------------------------------------------------- */
extern void HardFault_Handler();

void heartBeatLED();
void monitorStatus();
void PHAL_FaultHandler();
void preflightAnimation();
void preflightChecks();
void readCurrents();
void sendHeartBeat();
void updateTherm();


/**
 * Procedure: main()
 * 
 * @brief entry point
 * 
 */
int main(void)
{
    /* Temporary trim to offset clock drift effects */
    PHAL_trimHSI(HSI_TRIM_A_BOX);
    
    if (0 != PHAL_configureClockRates(&clock_config))
    {
        PHAL_FaultHandler();
    }

    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        PHAL_FaultHandler();
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

//TODO MCG: can we get some coherency about what we want in preflight vs here?
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_EnableIRQ(CAN2_RX0_IRQn); 
    initCANParse();
    orionInit();

//TODO MCG: why are we setting these to false they are already false is it because DAQ variables?
    bms_daq_override = false;
    bms_daq_stat = false;

    if (daqInit(&q_tx_can1_s[2]))
    {
        HardFault_Handler();
    }

//TODO MCG: Why are we doing this again? Which one do you actually want?
    /* See Datasheet DS11451 Figure. 4 for clock tree  */
    schedInit(APB1ClockRateHz * 2);     /* Initialize the scheduler */

    /* Task Creation */
    schedInit(SystemCoreClock);

    /* Preflight */
    configureAnim(preflightAnimation, preflightChecks, 75, 750);

    /* Periodic Tasks */
    taskCreate(heartBeatLED, 500);
    taskCreate(monitorStatus, 50);
    taskCreate(orionChargePeriodic, 50);
    taskCreate(heartBeatTask, 100);
    taskCreate(sendHeartBeat, 500);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);
    taskCreate(readCurrents, 50);

    /* Background Tasks */
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);

    /* Start all tasks */
    schedStart();

    return 0;

} /* main() */


/**
 * Procedure: a_box_bl_cmd_CALLBACK()
 * 
 * @brief Callback function for handling a_box bootloader commands.
 * 
 * This function processes received CAN messages related to the a_box bootloader commands.
 * If the received command is a reset command (`BLCMD_RST`), it triggers the bootloader reset 
 * for firmware download.
 * 
 * @param msg_data_a Pointer to the parsed CAN message data.
 */
void a_box_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.a_box_bl_cmd.cmd == BLCMD_RST)
    {
        Bootloader_ResetForFirmwareDownload();
    }

} /* a_box_bl_cmd_CALLBACK() */


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
 * Procedure: CAN2_RX0_IRQHandler()
 * 
 * @brief Interrupt handler for CAN2 RX0.
 * 
 * This function handles the interrupt request for CAN2 RX0 by calling the 
 * `canParseIRQHandler` function, passing the CAN1 instance as a parameter.
 */
void CAN2_RX0_IRQHandler()
{
    canParseIRQHandler(CAN2);
    
} /* CAN2_RX0_IRQHandler() */


/**
 * Procedure: heartBeatLED()
 * 
 * @brief Manages the heartbeat LED and connection LED based on CAN bus activity.
 * 
 * This function controls the state of the connection LED based on the time elapsed since the last CAN message reception.
 * If more than 500 milliseconds have passed since the last CAN message was received, the connection LED is turned off.
 * Otherwise, it is turned on. Additionally, the heartbeat LED is toggled to indicate activity.
 * Every other call to the function, it sends CAN statistics.
 * 
 */
void heartBeatLED()
{
    if ((sched.os_ticks - last_can_rx_time_ms) >= 500)
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    }
    else 
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
    }

    PHAL_toggleGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin);

    static uint8_t trig;
    if (trig) 
    {
        SEND_A_BOX_CAN_STATS(can_stats.tx_of, can_stats.tx_fail,
                    can_stats.rx_of, can_stats.rx_overrun);
    }
    trig = !trig;

} /* heartBeatLED() */


/**
 * Procedure: monitorStatus()
 * 
 * @brief Monitors and updates the status of the system based on various error conditions.
 * 
 * This function checks for errors from the BMS (Battery Management System), TMU (Temperature Measurement Unit),
 * and IMD (Insulation Monitoring Device). It updates the error LED and sets the fault status accordingly.
 * Additionally, it manages overrides for BMS and TMU data acquisition and updates the BMS status LED.
 * 
 */
void monitorStatus()
{
    uint8_t bms_err, imd_err, tmu_err;

    /* Check for errors from the BMS and TMU */
    bms_err = orionErrors();
    tmu_err = readTemps(&tmu);
    
    /* Check for an error from the IMD (active low) */
    imd_err = (false == PHAL_readGPIO(IMD_STATUS_GPIO_Port, IMD_STATUS_Pin));

    /* Handle overrides and update the error LED */
    if (bms_daq_override | tmu_daq_override) 
    {
        PHAL_toggleGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
    }
    else 
    {
        PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, bms_err);
    }

    /* Set the fault status for the IMD */
    setFault(ID_IMD_FAULT, imd_err);

    /* Determine the overall system status */
    uint8_t stat = bms_err | tmu_err;
    
    /* Apply BMS DAQ override if active */
    if (bms_daq_override)
    {
        stat = bms_daq_stat;
    }

    /* Update the BMS status LED */
    PHAL_writeGPIO(BMS_STATUS_GPIO_Port, BMS_STATUS_Pin, stat);

} /* monitorStatus() */


/**
 * Procedure: PHAL_FaultHandler()
 * 
 * @brief Handles faults by turning on the error LED, triggering a breakpoint, and calling the hard fault handler.
 * 
 */
void PHAL_FaultHandler()
{
    PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
    asm("bkpt");
    HardFault_Handler();

} /* PHAL_FaultHandler() */


/**
 * Procedure: preflightAnimation()
 * 
 * @brief Executes a preflight animation sequence for LEDs.
 * 
 * This function performs a simple preflight animation by sequentially turning on
 * the HEARTBEAT, CONNECTION, and ERROR LEDs. Each call to the function will update
 * the state of the LEDs based on a static time variable.
 * 
 */
void preflightAnimation(void)
{
   static uint32_t time = 0;

   PHAL_writeGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, 0);
   PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
   PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 0);

   switch (time++ % 3)
   {
       case 0:
           PHAL_writeGPIO(HEARTBEAT_LED_GPIO_Port, HEARTBEAT_LED_Pin, 1);
           break;
       case 1:
           PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
           break;
       case 2:
           PHAL_writeGPIO(ERROR_LED_GPIO_Port, ERROR_LED_Pin, 1);
           break;
   }
   
} /* preflightAnimation() */


/**
 * Procedure: preflightChecks()
 * 
 * @brief Performs preflight checks and initialization for various modules.
 */
void preflightChecks(void)
{
    static uint16_t state;
    uint8_t charger_speed_def = 0;

    switch (state++)
    {
        case 0:
            initTMU(&tmu);
            break;
        case 1:
            initFaultLibrary(FAULT_NODE_NAME, &q_tx_can1_s[0], ID_FAULT_SYNC_A_BOX);
            break;
        case 700:
            charger_speed_def = PHAL_readGPIO(BMS_CHARGE_ENABLE_Port, BMS_CHARGE_ENABLE_Pin);
//todo is this a required read? otherwise unused
            uint8_t speed_2 = PHAL_readGPIO(BMS_CHARGER_SAFETY_Port, BMS_CHARGER_SAFETY_Pin);
            if (charger_speed_def)
            {
                if (false == PHAL_initCAN(CAN1, false, VCAN_BPS))
                {
                    PHAL_FaultHandler();
                }
            }
            else
            {
                if (false == PHAL_initCAN(CAN1, false, CCAN_BPS))
                {
                    PHAL_FaultHandler();
                }
            }
            break;
        default:
            if (state > 750)
            {
                registerPreflightComplete(1);
                state = 751;
            }
    }

} /* preflightChecks() */


/**
 * Procedure: readCurrents()
 * 
 * @brief Reads current sensor values, calculates the currents, and sends the values over CAN.
 * 
 * This function reads ADC values from current sensors, calculates the actual currents using
 * the specified equations and parameters, and then sends the calculated current values over CAN.
 * 
 */
void readCurrents() {
    /* Storing ADC values to preserve them */
    uint16_t adc_isense_1 = adc_readings.isense_ch1;
    uint16_t adc_isense_2 = adc_readings.isense_ch2;
    
    /* Calculating currents from ADC using equation from: https://www.lem.com/sites/default/files/products_datasheets/dhab_s_124.pdf */
    float V_offset = 2.5;       /* Offset voltage (V)           */
    float G1 = 26.7 / 1000;     /* Channel 1 sensitivity (V/A)  */
    float G2 = 4.0 / 1000;      /* Channel 2 sensitivity (V/A)  */
    
    /* Calculating Vout and converting from 3.3V to 5V based on voltage divider */
    float Vout_ch1 = (ADC_VREF / ADC_ADDR_SIZE) * adc_isense_1 * (R1_ISENSE + R2_ISENSE) / R2_ISENSE;
    float Vout_ch2 = (ADC_VREF / ADC_ADDR_SIZE) * adc_isense_2 * (R1_ISENSE + R2_ISENSE) / R2_ISENSE;
    
    /* Calculating current, scaling by 17 due to coil turns, multiplying by 100 to send as int over CAN */
    int16_t i_ch1 = (Vout_ch1 - V_offset) / G1 * 100;
    int16_t i_ch2 = (Vout_ch2 - V_offset) / G2 * 100;
    
    SEND_I_SENSE(i_ch1, i_ch2);

} /* readCurrents() */


/**
 * Procedure: sendHeartBeat()
 * 
 * @brief Sends a heartbeat signal with the current IMD status and Orion error status.
 * 
 * This function reads the status of the IMD (Insulation Monitoring Device) from the specified GPIO port and pin.
 * It then sends a precharge heartbeat signal with the IMD status and the current Orion error status.
 */
void sendHeartBeat()
{
    bool imd_status = (false == PHAL_readGPIO(IMD_STATUS_GPIO_Port, IMD_STATUS_Pin));
    SEND_PRECHARGE_HB(imd_status, orion_error);

} /* sendHeartBeat() */
