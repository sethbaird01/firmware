/**
 * @file main.c
 * @author Purdue Electric Racing
 * @brief Main Module operation
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
#include "common/phal_F4_F7/usart/usart.h"
#include "common/plettenberg/plettenberg.h"
#include "common/psched/psched.h"
#include "common/queue/queue.h"

/* -------------------------------------------------------
    Module Includes 
-------------------------------------------------------- */
#include "can_parse.h"
#include "car.h"
#include "cooling.h"
#include "daq.h"
#include "main.h"

/* -------------------------------------------------------
    Pin Initialization
-------------------------------------------------------- */
GPIOInitConfig_t gpio_config[] =
{
    /* Internal Status Indicators */
    GPIO_INIT_OUTPUT(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_OUTPUT_LOW_SPEED),

    /* External Status Indicators */
    GPIO_INIT_OUTPUT(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(BRK_BUZZER_STAT_GPIO_Port, BRK_BUZZER_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_INPUT(TSAL_LVAL_STAT_GPIO_Port, TSAL_LVAL_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),

    /* CAN */
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,
    GPIO_INIT_CAN2RX_PB12,
    GPIO_INIT_CAN2TX_PB13,

    /* SPI */
    GPIO_INIT_SPI1_SCK_PA5,
    GPIO_INIT_SPI1_MISO_PA6,
    GPIO_INIT_SPI1_MOSI_PA7,
    GPIO_INIT_OUTPUT(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(EEPROM_nWP_GPIO_Port, EEPROM_nWP_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(EEPROM_NSS_GPIO_Port, EEPROM_NSS_Pin, GPIO_OUTPUT_LOW_SPEED),

    /* Shutdown Circuits */
    GPIO_INIT_OUTPUT(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S0_GPIO_Port, SDC_MUX_S0_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S1_GPIO_Port, SDC_MUX_S1_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S2_GPIO_Port, SDC_MUX_S2_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SDC_MUX_S3_GPIO_Port, SDC_MUX_S3_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_INPUT(SDC_MUX_DATA_GPIO_Port, SDC_MUX_DATA_Pin, GPIO_INPUT_OPEN_DRAIN),

    /* HV Bus Information */
    // GPIO_INIT_ANALOG(V_MC_SENSE_GPIO_Port, V_MC_SENSE_Pin),
    // GPIO_INIT_ANALOG(V_BAT_SENSE_GPIO_Port, V_BAT_SENSE_Pin),
    GPIO_INIT_INPUT(BMS_STAT_GPIO_Port, BMS_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_INPUT(PRCHG_STAT_GPIO_Port, PRCHG_STAT_Pin, GPIO_INPUT_OPEN_DRAIN),

    /* Motor Controllers */
    GPIO_INIT_USART2TX_PA2,
    GPIO_INIT_USART2RX_PA3,
    GPIO_INIT_USART1TX_PA9,
    GPIO_INIT_USART1RX_PA10,

    /* Wheel Speed */
    GPIO_INIT_AF(MOTOR_R_WS_GPIO_Port, MOTOR_R_WS_Pin, MOTOR_R_WS_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(MOTOR_L_WS_GPIO_Port, MOTOR_L_WS_Pin, MOTOR_L_WS_AF, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_DOWN),

    /* Shock Pots */
    GPIO_INIT_ANALOG(SHOCK_POT_L_GPIO_Port, SHOCK_POT_L_Pin),
    GPIO_INIT_ANALOG(SHOCK_POT_R_GPIO_Port, SHOCK_POT_R_Pin),

    /* Load Sensor */
    GPIO_INIT_ANALOG(LOAD_L_GPIO_Port, LOAD_L_Pin),
    GPIO_INIT_ANALOG(LOAD_R_GPIO_Port, LOAD_R_Pin),

    /* Thermistor Analog Multiplexer */
    GPIO_INIT_OUTPUT(THERM_MUX_S0_GPIO_Port, THERM_MUX_S0_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(THERM_MUX_S1_GPIO_Port, THERM_MUX_S1_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(THERM_MUX_S2_GPIO_Port, THERM_MUX_S2_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_ANALOG(THERM_MUX_D_GPIO_Port, THERM_MUX_D_Pin)
};

/* -------------------------------------------------------
    USART 
-------------------------------------------------------- */
q_handle_t q_tx_usart_l;    /* Global USART1 TX Queue         */
q_handle_t q_tx_usart_r;    /* Global USART2 TX Queue         */
uint16_t num_failed_msgs_l; /* Global failed msg count USART1 */
uint16_t num_failed_msgs_r; /* Global failed msg count USART2 */

uint8_t tmp_left[MC_MAX_TX_LENGTH] = {'\0'};  /* Command buffer for USART1 transmission */
uint8_t tmp_right[MC_MAX_TX_LENGTH] = {'\0'}; /* Command buffer for USART2 transmission */

/* -------------------------------------------------------
    USART1 DMA Configuration
-------------------------------------------------------- */
dma_init_t usart_l_tx_dma_config = USART1_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_l_rx_dma_config = USART1_RXDMA_CONT_CONFIG(NULL, 2);

/* -------------------------------------------------------
    USART1 Configuration (Left Motor Controller)
-------------------------------------------------------- */
usart_init_t huart_l =
{
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART1,
   .wake_addr = false,
   .usart_active_num = USART1_ACTIVE_IDX,
   .tx_dma_cfg = &usart_l_tx_dma_config,
   .rx_dma_cfg = &usart_l_rx_dma_config
};

/* -------------------------------------------------------
    USART1 RX Buffer
-------------------------------------------------------- */
char usart_l_rx_array[MC_MAX_RX_LENGTH] = {'\0'};
volatile usart_rx_buf_t huart_l_rx_buf =
{
    .last_msg_time = 0, .msg_size = MC_MAX_TX_LENGTH,
    .last_msg_loc  = 0, .last_rx_time = 0,
    .rx_buf_size   = MC_MAX_RX_LENGTH, .rx_buf = usart_l_rx_array
};

/* -------------------------------------------------------
    USART2 DMA Configuration
-------------------------------------------------------- */
dma_init_t usart_r_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_r_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);

/* -------------------------------------------------------
    USART2 Configuration (Right Motor Controller)
-------------------------------------------------------- */
usart_init_t huart_r =
{
   .baud_rate   = 115200,
   .word_length = WORD_8,
   .stop_bits   = SB_ONE,
   .parity      = PT_NONE,
   .hw_flow_ctl = HW_DISABLE,
   .ovsample    = OV_16,
   .obsample    = OB_DISABLE,
   .periph      = USART2,
   .wake_addr = false,
   .usart_active_num = USART2_ACTIVE_IDX,
   .tx_dma_cfg = &usart_r_tx_dma_config,
   .rx_dma_cfg = &usart_r_rx_dma_config
};

/* -------------------------------------------------------
    USART2 RX Buffer
-------------------------------------------------------- */
char usart_r_rx_array[MC_MAX_RX_LENGTH] = {'\0'};
volatile usart_rx_buf_t huart_r_rx_buf =
{
    .last_msg_time = 0, .msg_size = MC_MAX_TX_LENGTH,
    .last_msg_loc  = 0, .last_rx_time = 0,
    .rx_buf_size   = MC_MAX_RX_LENGTH, .rx_buf = usart_r_rx_array
};

/* -------------------------------------------------------
    ADC 
-------------------------------------------------------- */
volatile ADCReadings_t adc_readings;

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
    .adc_number      = 1
};

// TODO: Update this comment with the correct sample time
/* With 11 items, 16 prescaler, and 640 sample time, each channel gets read every 1.4ms */

/* -------------------------------------------------------
    ADC1 Channel Configuration
-------------------------------------------------------- */
ADCChannelConfig_t adc_channel_config[] =
{
    {.channel=V_MC_SENSE_ADC_CHNL,     .rank=1,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=V_BAT_SENSE_ADC_CHNL,    .rank=2,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SHOCK_POT_L_ADC_CHNL,    .rank=3,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=SHOCK_POT_R_ADC_CHNL,    .rank=4,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=THERM_MUX_D_ADC_CHNL,    .rank=5,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LOAD_L_ADC_CHNL,         .rank=6,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=LOAD_R_ADC_CHNL,         .rank=7,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
    {.channel=INTERNAL_THERM_ADC_CHNL, .rank=8,  .sampling_time=ADC_CHN_SMP_CYCLES_480},
};

/* -------------------------------------------------------
    ADC1 DMA Configuration
-------------------------------------------------------- */
dma_init_t adc_dma_config = ADC1_DMA_CONT_CONFIG((uint32_t) &adc_readings, sizeof(adc_readings) / sizeof(adc_readings.v_mc), 0b01);

/* -------------------------------------------------------
    Clock Rates
-------------------------------------------------------- */
extern uint32_t APB1ClockRateHz;        /* Defined in rcc.c */
extern uint32_t APB2ClockRateHz;        /* Defined in rcc.c */
extern uint32_t AHBClockRateHz;         /* Defined in rcc.c */
extern uint32_t PLLClockRateHz;         /* Defined in rcc.c */

/* -------------------------------------------------------
    Clock Configuration
-------------------------------------------------------- */
ClockRateConfig_t clock_config =
{
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =288000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
};

/* -------------------------------------------------------
    Procedures
-------------------------------------------------------- */
extern void HardFault_Handler();

void heartBeatLED();
void interpretLoadSensor(void);
void preflightAnimation(void);
void preflightChecks(void);
void send_fault(uint16_t, bool);
void usartIdleIRQ(volatile usart_init_t *huart, volatile usart_rx_buf_t *rx_buf);
void usartReceiveCompleteCallback(usart_init_t *handle);
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
    qConstruct(&q_tx_usart_l, MC_MAX_TX_LENGTH); /* TX queue for USART1 */
    qConstruct(&q_tx_usart_r, MC_MAX_TX_LENGTH); /* TX queue for USART2 */

    /* Temporary trim to offset clock drift effects */
    PHAL_trimHSI(HSI_TRIM_MAIN_MODULE);

    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }

    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    // TODO give reason for this
    PHAL_writeGPIO(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, 1);

    /* Initialize the scheduler */
    schedInit(APB1ClockRateHz);

    /* Preflight */
    configureAnim(preflightAnimation, preflightChecks, 60, 750);

    /* Periodic Tasks */
    taskCreate(coolingPeriodic, 50);
    taskCreate(heartBeatLED, 500);
    taskCreate(monitorSDCPeriodic, 20);
    taskCreate(carHeartbeat, 500);
    taskCreate(carPeriodic, 15);
    taskCreate(interpretLoadSensor, 15);
    taskCreate(updateSDCFaults, 300);
    taskCreate(heartBeatTask, 100);
    taskCreate(sendShockpotData, 15);
    taskCreate(parseMCDataPeriodic, MC_LOOP_DT);
    taskCreate(daqPeriodic, DAQ_UPDATE_PERIOD);

    /* Background Tasks */
    taskCreateBackground(canTxUpdate);
    taskCreateBackground(canRxUpdate);
    taskCreateBackground(usartTxUpdate);
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
 * @brief Handles the Hard Fault exception.
 * 
 * This function is called when a hard fault occurs. It turns on the error LED 
 * and enters an infinite loop to halt the system.
 */
void HardFault_Handler(void)
{
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
    while(1)
    {
        __asm__("nop");
    }

} /* HardFault_Handler() */


/**
 * @brief Toggles the heartbeat LED and manages the connection LED status.
 * 
 * This function toggles the heartbeat LED and updates the connection LED 
 * based on the last received CAN message time. It alternates between sending 
 * MCU status and main module CAN stats every other call.
 */
void heartBeatLED(void)
{
    static uint8_t trig;
    // TODO: fix HB LED
    // TODO MCG: this comment has been here for 2 years is it fixed yet?
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);
    }
    else
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
    }

    if (trig)
    {
        SEND_MCU_STATUS(sched.skips, (uint8_t) sched.fg_time.cpu_use, 
                        (uint8_t) sched.bg_time.cpu_use, sched.error);
    }
    else
    {
        SEND_MAIN_MODULE_CAN_STATS(can_stats.tx_of, can_stats.tx_fail,
                                   can_stats.rx_of, can_stats.rx_overrun);
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
    float force_load_l = voltToForce(adc_readings.load_l);
    float force_load_r = voltToForce(adc_readings.load_r);
    SEND_LOAD_SENSOR_READINGS(force_load_l, force_load_r);

} /* interpretLoadSensor() */


/**
 * Procedure: main_module_bl_cmd_CALLBACK()
 * 
 * @brief Callback function for handling main module bootloader commands.
 * 
 * This function processes received CAN messages related to the main module bootloader commands.
 * If the received command is a reset command (`BLCMD_RST`), it triggers the bootloader reset 
 * for firmware download.
 * 
 * @param msg_data_a Pointer to the parsed CAN message data.
 */
void main_module_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.main_module_bl_cmd.cmd == BLCMD_RST)
    {
        Bootloader_ResetForFirmwareDownload();
    }

} /* main_module_bl_cmd_CALLBACK()*/


/**
 * @brief Performs a preflight LED animation sequence.
 * 
 * Toggles the heartbeat, error, and connection LEDs in a 6-step sequence.
 * 
 * - Steps 0 and 5: Turn on the heartbeat LED.
 * - Steps 1 and 4: Turn on the connection LED.
 * - Steps 2 and 3: Turn on the error LED.
 */
void preflightAnimation(void)
{
    static uint32_t time;

    PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 0);
    PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 0);
    PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);

    switch (time++ % 6)
    {
        case 0:
        case 5:
            PHAL_writeGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, 1);
            break;
        case 1:
        case 4:
            PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
            break;
        case 2:
        case 3:
            PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
            break;
    }
} /* preflightAnimation() */


void preflightChecks(void)
{
    static uint8_t state;

    switch (state++)
    {
        case 0:
            huart_l.rx_dma_cfg->circular = true;
            if (false == PHAL_initUSART(&huart_l, APB2ClockRateHz))
            {
                HardFault_Handler();
            }
            huart_r.rx_dma_cfg->circular = true;
            if (false == PHAL_initUSART(&huart_r, APB1ClockRateHz))
            {
                HardFault_Handler();
            }
            break;
        case 1:
            if (false == PHAL_initCAN(CAN1, false, VCAN_BPS))
            {
                HardFault_Handler();
            }
            NVIC_EnableIRQ(CAN1_RX0_IRQn);
           break;
        case 2:
            if (false == PHAL_initADC(ADC1, &adc_config, adc_channel_config, sizeof(adc_channel_config) / sizeof(ADCChannelConfig_t)))
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
            /* Initial rx request */
            PHAL_usartRxDma(&huart_r,
                            (uint16_t *) huart_r_rx_buf.rx_buf,
                            huart_r_rx_buf.rx_buf_size, 1);
            PHAL_usartRxDma(&huart_l,
                            (uint16_t *) huart_l_rx_buf.rx_buf,
                            huart_l_rx_buf.rx_buf_size, 1);
            break;
        case 4:
            carInit();
            coolingInit();
            break;
       case 5:
            initCANParse();
            if (daqInit(&q_tx_can1_s[2]))
            {
                HardFault_Handler();
            }
            initFaultLibrary(FAULT_NODE_NAME, &q_tx_can1_s[0], ID_FAULT_SYNC_MAIN_MODULE);
           break;
        default:
            registerPreflightComplete(1);
            state = 255; /* prevent wrap around */
    }

} /* preflightChecks() */

//todo figure out if this is called
void usartIdleIRQ(volatile usart_init_t *huart, volatile usart_rx_buf_t *rx_buf)
{
    // TODO: check for overruns, framing errors, etc
    uint16_t new_loc = 0;
    rx_buf->last_rx_time = sched.os_ticks;
    new_loc = rx_buf->rx_buf_size - huart->rx_dma_cfg->stream->NDTR;      // extract last location from DMA
    if (new_loc == rx_buf->rx_buf_size) new_loc = 0;                        // should never happen
    else if (new_loc < rx_buf->last_rx_loc) new_loc += rx_buf->rx_buf_size; // wrap around
    if (new_loc - rx_buf->last_rx_loc > rx_buf->msg_size)                   // status msg vs just an echo
    {
        rx_buf->last_msg_time = sched.os_ticks;
        rx_buf->last_msg_loc = (rx_buf->last_rx_loc + 1) % rx_buf->rx_buf_size;
    }
    rx_buf->last_rx_loc = new_loc % rx_buf->rx_buf_size;

} /* usartIdleIRQ() */


// todo I can't find where this is/was attached to anything?
void usartReceiveCompleteCallback(usart_init_t *handle)
{
    if (handle == &huart_r)
    {
        if (handle->rx_errors.noise_detected)
        {
            num_failed_msgs_r++;
            return;
        }
        usartIdleIRQ(&huart_r, &huart_r_rx_buf);
    }
    else if (handle == &huart_l)
    {
        if (handle->rx_errors.noise_detected)
        {
            num_failed_msgs_l++;
            return;
        }
        usartIdleIRQ(&huart_l, &huart_l_rx_buf);
    }

} /* usartReceiveCompleteCallback() */


/**
 * Procedure: usartTxUpdate()
 * 
 * @brief Updates the USART transmission for the motor controllers.
 * 
 * This function checks if the USART transmission for each motor controller
 * is not busy and if there is a command to send from the transmission queue.
 * If both conditions are met for each of the motor controllers, it 
 * initiates a DMA transmission of the command.
 */
void usartTxUpdate(void)
{
    if (false == PHAL_usartTxBusy(&huart_l) &&
        qReceive(&q_tx_usart_l, tmp_left) == SUCCESS_G)
    {
        PHAL_usartTxDma(&huart_l, (uint16_t *) tmp_left, strlen(tmp_left));
    }
    if (false == PHAL_usartTxBusy(&huart_r) &&
        qReceive(&q_tx_usart_r, tmp_right) == SUCCESS_G)
    {
        PHAL_usartTxDma(&huart_r, (uint16_t *) tmp_right, strlen(tmp_right));
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
    return ((load_read / 4095.0 * 3.3) * (1 + (3.4/6.6))) * 100.0 * 9.8;

} /* voltToForce() */
