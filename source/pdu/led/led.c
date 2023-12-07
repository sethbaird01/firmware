#include "led.h"

#include "common/phal_F4_F7/spi/spi.h"
#include "common/phal_F4_F7/gpio/gpio.h"

void initLED(led_init_t *led)
{
    PHAL_writeGPIO(led->latch_port, led->latch_pin, 0);
    sendword(led, 0xFFFF);
    //Lower Blank pin to allow us to start messing with things
    PHAL_writeGPIO(led->blank_port, led->blank_pin, 0);

}


void toggleLatch(led_init_t *led)
{
    PHAL_writeGPIO(led->latch_port, led->latch_pin, 1);
    // im not putting in the work to figure out how long this will actually take, whoever
    // does do this driver for real should figure that out
    for (uint16_t i = 0; i < 1000; i++)
        ;
    PHAL_writeGPIO(led->latch_port, led->latch_pin, 0);
}

void sendword(led_init_t *led, uint16_t word)
{
    uint8_t write_buf[4] = {0};
    uint8_t read_buf[4] = {0};
    write_buf[0] =  (uint8_t) ((word & 0xFF00) >> 8);
    write_buf[1] = (uint8_t) ((word & 0x00FF));
    while (PHAL_SPI_busy(led->spi))
        ;
    PHAL_SPI_transfer(led->spi, write_buf, 2, read_buf);
    while (PHAL_SPI_busy(led->spi))
        ;
    toggleLatch(led);
}