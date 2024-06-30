/**
 * @file main.c
 * @author Purdue Electric Racing
 * @brief Torque Vectoring main operation
 * @version 0.1
 * @date 2024-05-28
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "common/bootloader/bootloader_common.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/psched/psched.h"
#include "common/phal_F4_F7/usart/usart.h"
#include "common/faults/faults.h"
#include "common/common_defs/common_defs.h"

/* Module Includes */
#include "bsxlite_interface.h"
#include "bmi088.h"
#include "gps.h"
#include "imu.h"
#include "source/torque_vector/can/can_parse.h"
#include "main.h"

// TODO I think these MUST be in a certain order...
#include "ac_ext.h"
#include "ac_compute_R.h"
#include "em.h"
#include "em_pp.h"
#include "tv.h"
#include "tv_pp.h"

/* -------------------------------------------------------
    Pin Initialization
-------------------------------------------------------- */
GPIOInitConfig_t gpio_config[] =
{
    /* Status Indicators */
    GPIO_INIT_OUTPUT(ERR_LED_GPIO_Port, ERR_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONN_LED_GPIO_Port, CONN_LED_Pin, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_OUTPUT_LOW_SPEED),

    /* SPI */
    GPIO_INIT_AF(SPI_SCLK_GPIO_Port, SPI_SCLK_Pin, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_PULL_DOWN),
    GPIO_INIT_AF(SPI_MISO_GPIO_Port, SPI_MISO_Pin, 5, GPIO_OUTPUT_HIGH_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_OPEN_DRAIN),
    GPIO_INIT_OUTPUT(SPI_CS_ACEL_GPIO_Port, SPI_CS_ACEL_Pin, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(SPI_CS_MAG_GPIO_Port, SPI_CS_MAG_Pin, GPIO_OUTPUT_HIGH_SPEED),

    /* GPS USART */
    GPIO_INIT_UART4RX_PA1,
    GPIO_INIT_UART4TX_PA0,

    /* GPS Auxiliary pins */
    GPIO_INIT_OUTPUT(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_OUTPUT_LOW_SPEED),

    /* EEPROM */
    GPIO_INIT_OUTPUT(NAV_EEPROM_CS_GPIO_PORT, NAV_EEPROM_CS_PIN, GPIO_OUTPUT_HIGH_SPEED),
    GPIO_INIT_OUTPUT(NAV_WP_GPIO_PORT, NAV_WP_PIN, GPIO_OUTPUT_HIGH_SPEED),

    /* CAN */
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12
};

/* -------------------------------------------------------
    UART
-------------------------------------------------------- */

/* -------------------------------------------------------
    UART4 DMA Configuration
-------------------------------------------------------- */
dma_init_t usart_gps_tx_dma_config = USART4_TXDMA_CONT_CONFIG(NULL, 1);
dma_init_t usart_gps_rx_dma_config = USART4_RXDMA_CONT_CONFIG(NULL, 2);

/* -------------------------------------------------------
    UART4 Configuration (GPS Communication)
-------------------------------------------------------- */
usart_init_t huart_gps =
{
    .baud_rate = 115200,
    .word_length = WORD_8,
    .hw_flow_ctl = HW_DISABLE,
    .stop_bits = SB_ONE,
    .parity = PT_NONE,
    .obsample = OB_DISABLE,
    .ovsample = OV_16,
    .periph      = UART4,
    .wake_addr = false,
    .usart_active_num = USART4_ACTIVE_IDX,
    .tx_errors = 0,
    .rx_errors = 0,
    .tx_dma_cfg = &usart_gps_tx_dma_config,
    .rx_dma_cfg = &usart_gps_rx_dma_config
};

/* -------------------------------------------------------
    Clock Configuration
-------------------------------------------------------- */
ClockRateConfig_t clock_config =
{
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =192000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
};

/* -------------------------------------------------------
    Clock Rates
-------------------------------------------------------- */
extern uint32_t APB1ClockRateHz;        /* Defined in rcc.c */
extern uint32_t APB2ClockRateHz;        /* Defined in rcc.c */
extern uint32_t AHBClockRateHz;         /* Defined in rcc.c */
extern uint32_t PLLClockRateHz;         /* Defined in rcc.c */


/* -------------------------------------------------------
    SPI1 Configuration (DMA)
-------------------------------------------------------- */
dma_init_t spi_rx_dma_config = SPI1_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI1_TXDMA_CONT_CONFIG(NULL, 1);

SPI_InitConfig_t spi_config =
{
    .data_rate = TargetCoreClockrateHz / 64,
    .data_len = 8,
    .nss_sw = true,
    .nss_gpio_port = SPI_CS_ACEL_GPIO_Port,
    .nss_gpio_pin = SPI_CS_ACEL_Pin,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI1
};

/* -------------------------------------------------------
    IMU, GPS, GYRO Data & Config
-------------------------------------------------------- */
GPS_Handle_t GPSHandle = {};
vector_3d_t accel_in, gyro_in, mag_in;

BMI088_Handle_t bmi_config =
{
    .accel_csb_gpio_port = SPI_CS_ACEL_GPIO_Port,
    .accel_csb_pin = SPI_CS_ACEL_Pin,
    .accel_range = ACCEL_RANGE_3G,
    .accel_odr = ACCEL_ODR_50Hz,
    .accel_bwp = ACCEL_OS_NORMAL,
    .gyro_csb_gpio_port = SPI_CS_GYRO_GPIO_Port,
    .gyro_csb_pin = SPI_CS_GYRO_Pin,
    .gyro_datarate = GYRO_DR_100Hz_32Hz,
    .gyro_range = GYRO_RANGE_250,
    .spi = &spi_config
};

IMU_Handle_t imu_h =
{
    .bmi = &bmi_config,
};

/* Torque Vectoring Definitions */
static ExtU_tv rtU_tv;                          /* External inputs   */
static ExtY_tv rtY_tv;                          /* External outputs  */
static RT_MODEL_tv rtM_tvv;
static RT_MODEL_tv *const rtMPtr_tv = &rtM_tvv; /* Real-time model   */
static DW_tv rtDW_tv;                           /* Observable states */
static RT_MODEL_tv *const rtM_tv = rtMPtr_tv;
static int16_t tv_timing;

/* Engine Map Definitions */
static ExtU_em rtU_em;                          /* External inputs   */
static ExtY_em rtY_em;                          /* External outputs  */
static RT_MODEL_em rtM_emv;
static RT_MODEL_em *const rtMPtr_em = &rtM_emv; /* Real-time model   */
static DW_em rtDW_em;                           /* Observable states */
static RT_MODEL_em *const rtM_em = rtMPtr_em;
static int16_t em_timing;

/* Moving Median Definition */
static vec_accumulator vec_mm;      /* Vector of Accumulation                         */
static int16_t ac_counter = 0;      /* Number of data points collected                */
static bool TV_Calibrated = false;  /* Flag Indicating if TV is calibrated            */
static int16_t gyro_counter = 0;    /* Number of steps that gyro has not been checked */

/* -------------------------------------------------------
    Procedures
-------------------------------------------------------- */
extern void HardFault_Handler(void);
void heartBeatLED(void);
void preflightAnimation(void);
void preflightChecks(void);
void parseIMU(void);
void pollIMU(void);
void VCU_MAIN(void);


/**
 * Procedure: main()
 * 
 * @brief entry point
 */
int main(void)
{
    /* Temporary trim to offset clock drift effects */
    PHAL_trimHSI(HSI_TRIM_TORQUE_VECTOR);

    if (0 != PHAL_configureClockRates(&clock_config))
    {
        HardFault_Handler();
    }

    /* GPIO initialization */
    if (false == PHAL_initGPIO(gpio_config, sizeof(gpio_config) / sizeof(GPIOInitConfig_t)))
    {
        HardFault_Handler();
    }

    /* Task Creation */
    schedInit(APB1ClockRateHz);

    /* Preflight */
    configureAnim(preflightAnimation, preflightChecks, 74, 1000);

    /* Periodic Tasks */
    taskCreate(heartBeatLED, 500);
    taskCreate(heartBeatTask, 100);
    taskCreate(parseIMU, 20);
    taskCreate(pollIMU, 20);
    taskCreate(VCU_MAIN, 15);

    /* Background Tasks */
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
 * @brief Control heartbeat and connection LEDs based on system status.
 * 
 * This function toggles the heartbeat LED and controls the connection LED based on
 * the time elapsed since the last CAN message reception. It alternates sending
 * torque vectoring CAN statistics periodically.
 *
 */
void heartBeatLED(void)
{
    PHAL_toggleGPIO(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);

    if ((sched.os_ticks - last_can_rx_time_ms) >= CONN_LED_MS_THRESH)
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 0);   
    }
    else
    {
        PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
    }

    static uint8_t trig;
    if (trig)
    {
        SEND_TV_CAN_STATS(can_stats.tx_of, can_stats.tx_fail, can_stats.rx_of, can_stats.rx_overrun);
    }
    trig = !trig;

} /* heartBeatLED() */


/**
 * @brief Parses IMU data including gyro and accelerometer readings.
 */
void parseIMU(void)
{
    GPSHandle.messages_received++;
    BMI088_readGyro(&bmi_config, &gyro_in);
    BMI088_readAccel(&bmi_config, &accel_in);
    GPSHandle.acceleration = accel_in;
    GPSHandle.gyroscope = gyro_in;

    /* Update Gyro OK flag */
    if (gyro_counter == 150)
    {
        GPSHandle.gyro_OK = BMI088_gyroOK(&bmi_config);
        gyro_counter = 0;
    }
    else
    {
        ++gyro_counter;
    }

} /* parseIMU() */


/**
 * @brief Polls the IMU for periodic updates.
 */
void pollIMU(void)
{
    imu_periodic(&imu_h);
    
} /* pollIMU() */


/**
 * @brief Perform pre-flight animation on status indicators.
 * 
 * This function toggles LEDs connected to status indicators in a predefined pattern
 * to indicate system readiness or status during pre-flight checks.
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
        case 2:
        case 3:
            PHAL_writeGPIO(ERR_LED_GPIO_Port, ERR_LED_Pin, 1);
            break;
        case 4:
            PHAL_writeGPIO(CONN_LED_GPIO_Port, CONN_LED_Pin, 1);
            break;
    }

} /* preflightAnimation() */


/**
 * @brief Perform pre-flight checks and initialization tasks.
 * 
 * This function executes a series of initialization steps based on the current state.
 * It initializes CAN, SPI, USART, sensors, and other subsystems necessary for system startup.
 * 
 */
void preflightChecks(void)
{
    static uint16_t state;

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
        if (false == PHAL_SPI_init(&spi_config))
        {
            HardFault_Handler();
        }
        spi_config.data_rate = APB2ClockRateHz / 16;

        PHAL_writeGPIO(SPI_CS_ACEL_GPIO_Port, SPI_CS_ACEL_Pin, 1);
        PHAL_writeGPIO(SPI_CS_GYRO_GPIO_Port, SPI_CS_GYRO_Pin, 1);
        PHAL_writeGPIO(SPI_CS_MAG_GPIO_Port, SPI_CS_MAG_Pin, 1);
        break;
    case 2:
        if (false == PHAL_initUSART(&huart_gps, APB1ClockRateHz))
        {
            HardFault_Handler();
        }
        break;
    case 3:
        PHAL_writeGPIO(GPS_RESET_GPIO_Port, GPS_RESET_Pin, 1);
        PHAL_usartRxDma(&huart_gps, (uint16_t *)GPSHandle.raw_message, 100, 1);
        break;
    case 4:
        if (!BMI088_init(&bmi_config))
        {
            HardFault_Handler();
        }
        break;
    case 5:
        initFaultLibrary(FAULT_NODE_NAME, &q_tx_can1_s[0], ID_FAULT_SYNC_TORQUE_VECTOR);
        break;
    case 250:
        BMI088_powerOnAccel(&bmi_config);
        break;
    case 500:
        if (false == BMI088_initAccel(&bmi_config))
        {
            HardFault_Handler();
        }
        break;
    case 700:
        /* Pack torque vectoring data into rtM_tv */
        rtM_tv->dwork = &rtDW_tv;

        /* Initialize Torque Vectoring */
        tv_initialize(rtM_tv);

        /* Initialize TV IO */
        tv_IO_initialize(&rtU_tv);

        /* Pack Engine map data into rtM_em */
        rtM_em->dwork = &rtDW_em;

        /* Initialize Engine Map */
        em_initialize(rtM_em);
        break;
    default:
        if (state > 750)
        {
            if (false == imu_init(&imu_h))
            {
                HardFault_Handler();
            }
            initCANParse();
            registerPreflightComplete(1);
            state = 750; /* prevent wrap around */
        }
        break;
    }

} /* preflightChecks() */


/**
 * Procedure: torquevector_bl_cmd_CALLBACK()
 * 
 * @brief Callback function for handling torque vectoring bootloader commands.
 * 
 * This function processes received CAN messages related to the torque vectoring bootloader commands.
 * If the received command is a reset command (`BLCMD_RST`), it triggers the bootloader reset 
 * for firmware download.
 * 
 * @param msg_data_a Pointer to the parsed CAN message data.
 */
void torquevector_bl_cmd_CALLBACK(CanParsedData_t *msg_data_a)
{
    if (can_data.torquevector_bl_cmd.cmd == BLCMD_RST)
    {
        Bootloader_ResetForFirmwareDownload();
    }

} /* torquevector_bl_cmd_CALLBACK() */


/**
 * @brief USART receive complete callback.
 *
 * Parses velocity data from GPS after USART receive.
 *
 * @param handle Pointer to USART initialization.
 */
void usart_recieve_complete_callback(usart_init_t *handle)
{
    parseVelocity(&GPSHandle);

} /* usart_recieve_complete_callback() */


/**
 * @brief Main function for Vehicle Control Unit (VCU) operations.
 * 
 * This function manages various tasks related to vehicle control,
 * including throttle initialization, calibration, torque vectoring,
 * engine map processing, fault management, and message sending.
 */
void VCU_MAIN(void)
{
    /* Initialize Throttles */
    int16_t tvs_k_rl = 0;
    int16_t tvs_k_rr = 0;
    int16_t equal_k_rl = 0;
    int16_t equal_k_rr = 0;

    /* If precharging -> Accumulate acceleration vector */
    if (ac_counter <= NUM_ELEM_ACC_CALIBRATION)
    {
        /* Accumulate acceleration vector */
        vec_mm.ax[ac_counter] = GPSHandle.acceleration.x;
        vec_mm.ay[ac_counter] = GPSHandle.acceleration.y;
        vec_mm.az[ac_counter] = GPSHandle.acceleration.z;
        ++ac_counter;

        /* If length of acceleration vector == ##, compute R */
        if (ac_counter == NUM_ELEM_ACC_CALIBRATION)
        {
            ac_compute_R(vec_mm.ax, vec_mm.ay, vec_mm.az, rtU_tv.R);
            TV_Calibrated = true;
        }
    }

    /* Populate torque vectoring inputs */
    tv_pp(&rtU_tv, &GPSHandle);

    /* Step torque vectoring */
    tv_timing = sched.os_ticks;
    tv_step(rtMPtr_tv, &rtU_tv, &rtY_tv);
    tv_timing = sched.os_ticks - tv_timing;

    /* Populate engine map inputs */
    em_pp(&rtU_em, &rtY_tv);

    /* Step engine map */
    em_timing = sched.os_ticks;
    em_step(rtMPtr_em, &rtU_em, &rtY_em);
    em_timing = sched.os_ticks - em_timing;

    /* Set TV faults */
    setFault(ID_TV_DISABLED_FAULT,!rtY_tv.TVS_STATE);
    setFault(ID_MM_DISABLED_FAULT,!rtY_em.MM_STATE);
    setFault(ID_TV_UNCALIBRATED_FAULT,!TV_Calibrated);
    setFault(ID_NO_GPS_FIX_FAULT,!rtU_tv.F_raw[8]);
    
    setFault(ID_TV_ENABLED_FAULT,rtY_tv.TVS_STATE);
    setFault(ID_MM_ENABLED_FAULT,rtY_em.MM_STATE);
    setFault(ID_TV_CALIBRATED_FAULT,TV_Calibrated);
    setFault(ID_YES_GPS_FIX_FAULT,rtU_tv.F_raw[8]);

    /* Get motor commands */
    tvs_k_rl = (int16_t)(rtY_em.kTVS[0]*4095);
    tvs_k_rr = (int16_t)(rtY_em.kTVS[1]*4095);
    equal_k_rl = (int16_t)(rtY_em.kEQUAL[0]*4095);
    equal_k_rr = (int16_t)(rtY_em.kEQUAL[1]*4095);

    /* Send messages */
    SEND_THROTTLE_VCU(tvs_k_rl,tvs_k_rr);
    SEND_MAXR((int16_t)(rtY_tv.max_K*4095));

    SEND_SFS_ACC((int16_t)(rtY_tv.sig_filt[15] * 100),(int16_t)(rtY_tv.sig_filt[16] * 100), (int16_t)(rtY_tv.sig_filt[17] * 100));
    SEND_SFS_ANG_VEL((int16_t)(rtY_tv.sig_filt[7] * 10000),(int16_t)(rtY_tv.sig_filt[8] * 10000), (int16_t)(rtY_tv.sig_filt[9] * 10000));

} /* VCU_MAIN() */
