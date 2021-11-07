#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_misc.h>
#include <espressif/esp_system.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include "i2s_dma/i2s_dma.h"
#include <udplogger.h>

#ifndef VERSION
 #error You must set VERSION=x.y.z to match github version tag x.y.z
#endif

#define  COIL1_PIN 4
#define  COIL2_PIN 5
//SPIKE_PIN is GPIO3 = RX0 because hardcoded in i2s
//DETECT_PIN is ADC
#define  ONES  0xffffffff // 32 bits of value one
#define  ZEROS 0x00000000 // 32 bits of value zero
#define  BUFSIZE 16       // must be multiple of 4
uint32_t dma_buf[BUFSIZE];
static   dma_descriptor_t dma_block;

void spike_task(void *argv) {
    //SPIKE_PIN is GPIO3 = RX0 because hardcoded in i2s - remove UART cable from RX0 port!
    i2s_pins_t i2s_pins = {.data = true, .clock = false, .ws = false};
    i2s_clock_div_t clock_div = i2s_get_clock_div(53333333); // 53MHz provides div=3 and 0.01875 microseconds per step
    i2s_dma_init(NULL, NULL, clock_div, i2s_pins);
    dma_block.owner = 1; dma_block.sub_sof = 0; dma_block.unused = 0;
    dma_block.next_link_ptr = 0; dma_block.eof = 1; //only one block with EOF so will be a single shot
    dma_block.datalen = BUFSIZE*4; dma_block.blocksize = BUFSIZE*4; // uint32_t is 4byte data x BUFSIZE words
    dma_block.buf_ptr = dma_buf;
    dma_buf[0]=ONES; dma_buf[1]=ONES;dma_buf[2]=~(ONES>>19); //83 bits gets us 321 kHz
    dma_buf[3]=ZEROS;dma_buf[4]=ZEROS;dma_buf[5]=ONES>>6; // we generate 3 pulses of 1.56 microseconds
    dma_buf[6]=ONES;dma_buf[7]=~(ONES>>25);dma_buf[8]=ZEROS;dma_buf[9]=ZEROS;
    dma_buf[10]=ONES>>12;dma_buf[11]=ONES;dma_buf[12]=~(ONES>>31);
    dma_buf[13]=ZEROS;dma_buf[14]=ZEROS;dma_buf[15]=ZEROS; //need to fill multiple of 4 32 bits words, so 16
    while (1) {
        printf("base %4d   ",sdk_system_adc_read());fflush(stdout);
        //because GPIO3=I2S output is LOW in rest between shots, we must generate a HIGH pulse.
        gpio_write(COIL1_PIN, 0); //enable COIL1
        sdk_os_delay_us(20); //stabilise the output?
        i2s_dma_start(&dma_block); //transmit the dma_buf once
        sdk_os_delay_us(60);
        printf("C1 %4d   ",sdk_system_adc_read());fflush(stdout);
        gpio_write(COIL1_PIN, 1); //disable COIL1
        
        vTaskDelay(25); //250ms
        
        gpio_write(COIL2_PIN, 0); //enable COIL2
        sdk_os_delay_us(20); //stabilise the output?
        i2s_dma_start(&dma_block); //transmit the dma_buf once
        sdk_os_delay_us(60);
        printf("C2 %4d\n",sdk_system_adc_read());
        gpio_write(COIL2_PIN, 1); //disable COIL2
        
        vTaskDelay(25); //250ms
    }
}

void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(3);
    UDPLUS("\n\n\nWaterMeter " VERSION "\n");
    gpio_enable( COIL1_PIN, GPIO_OUTPUT); gpio_write( COIL1_PIN, 1);
    gpio_enable( COIL2_PIN, GPIO_OUTPUT); gpio_write( COIL2_PIN, 1);
    xTaskCreate(spike_task, "Spike", 512, NULL, 1, NULL);
}
