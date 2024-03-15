#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/dma/dma.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#include "common/phal_F4_F7/rtc/rtc.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/usart/usart.h"

#include "buffer.h"
#include "main.h"
#include "sdio.h"
#include "daq_hub.h"
#include "common/modules/Wiznet/W5500/Ethernet/wizchip_conf.h"
#include "common/modules/Wiznet/W5500/Ethernet/socket.h"

#include "ff.h"

GPIOInitConfig_t gpio_config[] = {
    GPIO_INIT_OUTPUT(HEARTBEAT_LED_PORT, HEARTBEAT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(CONNECTION_LED_PORT, CONNECTION_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(ERROR_LED_PORT, ERROR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_ACTIVITY_LED_PORT, SD_ACTIVITY_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_ERROR_LED_PORT, SD_ERROR_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT(SD_DETECT_LED_PORT, SD_DETECT_LED_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_SPI2_SCK_PB13,
    GPIO_INIT_SPI2_MISO_PC2,
    GPIO_INIT_SPI2_MOSI_PC3,
    GPIO_INIT_OUTPUT(ETH_CS_PORT, ETH_CS_PIN, GPIO_OUTPUT_HIGH_SPEED),
    // GPIO_INIT_OUTPUT(ETH_RST_PORT, ETH_RST_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_OUTPUT_OPEN_DRAIN(ETH_RST_PORT, ETH_RST_PIN, GPIO_OUTPUT_LOW_SPEED),
    GPIO_INIT_SDIO_CLK,
    GPIO_INIT_SDIO_CMD,
    GPIO_INIT_SDIO_DT0,
    GPIO_INIT_SDIO_DT1,
    GPIO_INIT_SDIO_DT2,
    GPIO_INIT_SDIO_DT3,
    GPIO_INIT_INPUT(SD_CD_PORT, SD_CD_PIN, GPIO_INPUT_PULL_UP),
    GPIO_INIT_INPUT(LOG_ENABLE_PORT, LOG_ENABLE_PIN, GPIO_INPUT_PULL_UP),
    GPIO_INIT_USART2TX_PA2,
    GPIO_INIT_USART2RX_PA3,
    GPIO_INIT_INPUT(PWR_LOSS_PORT, PWR_LOSS_PIN, GPIO_INPUT_OPEN_DRAIN),
#ifdef DISCO_BOARD
    GPIO_INIT_CANRX_PD0,
    GPIO_INIT_CANTX_PD1,
#else
    GPIO_INIT_CANRX_PA11,
    GPIO_INIT_CANTX_PA12,
#endif
#ifdef EN_CAN2
    // TODO: CAN2 gpio pins
#endif
};

/* SPI CONFIG FOR ETHERNET MODULE */
dma_init_t spi_rx_dma_config = SPI2_RXDMA_CONT_CONFIG(NULL, 2);
dma_init_t spi_tx_dma_config = SPI2_TXDMA_CONT_CONFIG(NULL, 1);
SPI_InitConfig_t eth_spi_config = {
    .data_rate = 36000000 / 36,
    .data_len  = 8,
    .nss_sw = false,
    .nss_gpio_port = ETH_CS_PORT,
    .nss_gpio_pin = ETH_CS_PIN,
    .rx_dma_cfg = &spi_rx_dma_config,
    .tx_dma_cfg = &spi_tx_dma_config,
    .periph = SPI2
};

RTC_timestamp_t start_time = 
{
    .date = {.month_bcd=RTC_MONTH_FEBRUARY,
             .weekday=RTC_WEEKDAY_TUESDAY,
             .day_bcd=0x27,
             .year_bcd=0x24},
    .time = {.hours_bcd=0x18,
             .minutes_bcd=0x27,
             .seconds_bcd=0x00,
             .time_format=RTC_FORMAT_24_HOUR},
};

/* CLOCK CONFIG */
extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

#define TargetCoreClockrateHz 144000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_PLL,
    .pll_src                    =PLL_SRC_HSI16,
    .vco_output_rate_target_hz  =288000000,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / 4),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / 4),
};

volatile uint32_t tick_ms; // Systick 1ms counter

/* UART CONFIG FOR DEBUG PRINTS */
// dma_init_t usart_tx_dma_config = USART2_TXDMA_CONT_CONFIG(NULL, 1);
// dma_init_t usart_rx_dma_config = USART2_RXDMA_CONT_CONFIG(NULL, 2);
// usart_init_t uart_log_config = {
//    .baud_rate   = 115200,
//    .word_length = WORD_8,
//    .stop_bits   = SB_ONE,
//    .parity      = PT_NONE,
//    .mode        = MODE_TX_RX,
//    .hw_flow_ctl = HW_DISABLE,
//    .ovsample    = OV_16,
//    .obsample    = OB_DISABLE,
//    .adv_feature = {
//                    .wake_addr = false,
//                    .tx_inv = false,
//                    .rx_inv = false,
//                    .dma_on_rx_err = false,
//                   },
//    .tx_dma_cfg = &usart_tx_dma_config,
//    .rx_dma_cfg = &usart_rx_dma_config
// };

// For logging, requires message to have '\n'
#ifdef DEBUG_LOG
char log_buffer[100];
void _log_str(char* data)
{
    size_t len = 0;
    if (!data) return;
    while (data[len]) len++;
    if (len > 0)
    {
        // PHAL_usartTxDma(USART2, &uart_log_config, (uint16_t *)data, len);
        // while (!PHAL_usartTxDmaComplete(&uart_log_config));
    }
}
#endif

static void cs_sel(void);
static void cs_desel(void);
static uint8_t spi_rb(void);
static void spi_wb(uint8_t b);
static void spi_rb_burst(uint8_t *pBuf, uint16_t len);
static void spi_wb_burst(uint8_t *pBuf, uint16_t len);

q_handle_t q_tx_can;

volatile timestamped_frame_t rx_buffer[RX_BUFF_ITEM_COUNT];
b_tail_t tails[RX_TAIL_COUNT];
b_handle_t b_rx_can = {
    .buffer=(volatile uint8_t *)rx_buffer,
    .tails=tails,
    .num_tails=RX_TAIL_COUNT,
};

tcp_can_frame_t tcp_rx_buffer[TCP_RX_BUFF_ITEM_COUNT];
b_tail_t tcp_tails[TCP_RX_TAIL_COUNT];
b_handle_t b_rx_tcp = {
    .buffer=(uint8_t *)tcp_rx_buffer,
    .tails=tcp_tails,
    .num_tails=TCP_RX_TAIL_COUNT,
};

int main()
{

    // TODO: use watchdog to recover if timed out?
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    bConstruct(&b_rx_can, sizeof(*rx_buffer), sizeof(rx_buffer));
    bConstruct(&b_rx_tcp, 1, sizeof(tcp_rx_buffer)); // Byte resolution for tcp receive

    // TODO: size the buffer to handle the time it takes to write new file (last max loop time was 71ms)
    // TODO: investigate flushing file periodically to get the loop times down

    if(0 != PHAL_configureClockRates(&clock_config))
        HardFault_Handler();

    if(!PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        HardFault_Handler();

    if (!PHAL_SPI_init(&eth_spi_config))
        HardFault_Handler();

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);
    
    if (!PHAL_configureRTC(&start_time, true))
        HardFault_Handler();


#ifdef DEBUG_LOG
    // if(!PHAL_initUSART(USART2, &uart_log_config, APB1ClockRateHz))
    //     HardFault_Handler();
#endif

    log_yellow("PER PER PER\n");

    if(!PHAL_initCAN(CAN1, false))
        HardFault_Handler();
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

#ifdef EN_CAN2
    if(!PHAL_initCAN(CAN2, false))
        HardFault_Handler();
    NVIC_EnableIRQ(CAN2_RX0_IRQn);
#endif

    // Link SPI for ethernet driver
    PHAL_writeGPIO(ETH_CS_PORT, ETH_CS_PIN, 1);
    reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
    reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
    reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);

    daq_init();

    log_msg("Starting main loop\n");

    daq_loop();

    log_red("Main loop exited!\n");

    return 0;
}

void SysTick_Handler(void)
{
    tick_ms++;
}

/* SPI Callbacks for Ethernet Driver */

static void cs_sel(void)   {PHAL_writeGPIO(ETH_CS_PORT, ETH_CS_PIN, 0);}
static void cs_desel(void) {PHAL_writeGPIO(ETH_CS_PORT, ETH_CS_PIN, 1);}

static uint8_t spi_rb(void)
{
    uint8_t b;
    PHAL_SPI_transfer_noDMA(&eth_spi_config, NULL, 0, sizeof(b), &b);
    return b;
}

static void spi_wb(uint8_t b)
{
    PHAL_SPI_transfer_noDMA(&eth_spi_config, &b, sizeof(b), 0, NULL);
}

static void spi_rb_burst(uint8_t *pBuf, uint16_t len)
{
    // SPI RX Burst, must block! (uses local pointer)
    PHAL_SPI_transfer_noDMA(&eth_spi_config, NULL, 0, len, pBuf);
}

static void spi_wb_burst(uint8_t *pBuf, uint16_t len)
{
    // SPI TX Burst, must block! (uses local pointer)
    PHAL_SPI_transfer_noDMA(&eth_spi_config, pBuf, len, 0, NULL);
}

static void can_rx_irq_handler(CAN_TypeDef * can_h)
{
    // TODO: track FIFO overrun and full errors
    if (can_h->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        can_h->RF0R &= !(CAN_RF0R_FOVR0);

    if (can_h->RF0R & CAN_RF0R_FULL0) // FIFO Full
        can_h->RF0R &= !(CAN_RF0R_FULL0);

    if (can_h->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        timestamped_frame_t *rx;
        uint32_t cont;
        if (bGetHeadForWrite(&b_rx_can, (void**) &rx, &cont) == 0)
        {
            rx->tick_ms = tick_ms;

            rx->bus_id = (can_h == CAN1) ? BUS_ID_CAN1 : BUS_ID_CAN2;

            // Get either StdId or ExtId
            if (CAN_RI0R_IDE & can_h->sFIFOMailBox[0].RIR)
            {
                rx->msg_id = CAN_EFF_FLAG | (((CAN_RI0R_EXID | CAN_RI0R_STID) & can_h->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos);
            }
            else
            {
                rx->msg_id = (CAN_RI0R_STID & can_h->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
            }

            rx->dlc = (CAN_RDT0R_DLC & can_h->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

            rx->data[0] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 0)  & 0xFF;
            rx->data[1] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 8)  & 0xFF;
            rx->data[2] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
            rx->data[3] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
            rx->data[4] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 0)  & 0xFF;
            rx->data[5] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 8)  & 0xFF;
            rx->data[6] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
            rx->data[7] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

            bCommitWrite(&b_rx_can, 1);
        }
        can_h->RF0R |= (CAN_RF0R_RFOM0);
    }
}

void CAN1_RX0_IRQHandler()
{
    can_rx_irq_handler(CAN1);
}

#ifdef EN_CAN2
void CAN2_RX0_IRQHandler()
{
    can_rx_irq_handler(CAN2);
}
#endif

void HardFault_Handler()
{
    PHAL_writeGPIO(ERROR_LED_PORT, ERROR_LED_PIN, 1);
    while(1)
    {
        __asm__("nop");
    }
}
