#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_misc.h>
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

#define  TRIGGER_PIN 2
#define  ONES  0xffffffff // 32 bits of value one
#define  ZEROS 0x00000000 // 32 bits of value zero
#define  BUFSIZE 16        // with BUFSIZE = 1 the one shot doesn't work, must be multiple of 4
uint32_t dma_buf[BUFSIZE];
static   dma_descriptor_t dma_block;

void spike_task(void *argv) {
    int i,j=0;
    vTaskDelay(200);
    printf("remove UART cable from RXD port!\n");
    vTaskDelay(1000); // remove serial cable in these 10 seconds
    //SPIKE_PIN is GPIO3 = RX0 because hardcoded in i2s
    i2s_pins_t i2s_pins = {.data = true, .clock = false, .ws = false};
    i2s_clock_div_t clock_div = i2s_get_clock_div(53333333); // 53MHz provides div=3 and 0.01875 microseconds per step
    i2s_dma_init(NULL, NULL, clock_div, i2s_pins);
    dma_block.owner = 1; dma_block.sub_sof = 0; dma_block.unused = 0;
    dma_block.next_link_ptr = 0; dma_block.eof = 1; //only one block with EOF so will be a single shot
    dma_block.datalen = BUFSIZE*4; dma_block.blocksize = BUFSIZE*4; // uint32_t is 4byte data x BUFSIZE words
    dma_block.buf_ptr = dma_buf;
    dma_buf[0]=ONES; dma_buf[1]=ONES; dma_buf[3]=ZEROS; //dma_buf[2] is the tuning value
    while (1) {
        dma_buf[2]=~(ONES>>19);dma_buf[3]=ZEROS;dma_buf[4]=ZEROS;dma_buf[5]=ONES>>6;
        dma_buf[6]=ONES;dma_buf[7]=~(ONES>>25);dma_buf[8]=ZEROS;dma_buf[9]=ZEROS;
        dma_buf[10]=ONES>>12;dma_buf[11]=ONES;dma_buf[12]=~(ONES>>31);
        dma_buf[13]=ZEROS;dma_buf[14]=ZEROS;dma_buf[15]=ZEROS;
        while (1) { //experiment to output 3 fixed pulses without too much change in existing structure
            gpio_write(TRIGGER_PIN, 0);
            sdk_os_delay_us(16); //helps to trigger scope externally
            gpio_write(TRIGGER_PIN, 1);
            i2s_dma_start(&dma_block); //transmit the dma_buf once
            sdk_os_delay_us(100);
            printf("%4d\n",sdk_system_adc_read());
            vTaskDelay(50); //500ms
        }
        for (i=15;i<24;i++) { //best value is supposed to be 19 -> 1.55625 microseconds ~ 321kHz
             //TODO: WARNING, at BOOT, GPIO3 is HIGH for 10-50us, so maybe use another port to inhibit it?
            dma_buf[2]=~(ONES>>i); //because GPIO3=I2S output is LOW in rest between shots, we must generate a HIGH pulse.
            printf("spike=%d/32=0x%04x => t=%5.3f microseconds or %d kHz\n",i,dma_buf[0],(i+64)*3/160.0,80000/((i+64)*3));
            for (j=0;j<25;j++) { //repeat for 5 seconds
                printf(".");fflush(stdout);
                gpio_write(TRIGGER_PIN, 0);
                sdk_os_delay_us(16); //helps to trigger scope externally
                gpio_write(TRIGGER_PIN, 1);
                i2s_dma_start(&dma_block); //transmit the dma_buf once
                vTaskDelay(20); //200ms
                printf("+");fflush(stdout);
            }
            printf("\n");
        }
    }
}

void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(3);
    UDPLUS("\n\n\nWaterMeter " VERSION "\n");
    gpio_enable( TRIGGER_PIN, GPIO_OUTPUT); gpio_write( TRIGGER_PIN, 1);
    xTaskCreate(spike_task, "Spike", 512, NULL, 1, NULL);
}
