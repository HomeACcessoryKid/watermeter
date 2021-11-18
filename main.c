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
#include <lwip/apps/sntp.h>

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
#define  WINDOW  20       // minimum value of # of samples
#define  OFFSET  -8       // how much coil1 is higher than coil2
#define  HYSTERESIS  15   // to prevent noise to trigger
uint32_t dma_buf[BUFSIZE];
static   dma_descriptor_t dma_block;
time_t ts;

void spike_task(void *argv) {
    int i=0;
    bool direction=0;
    uint32_t halflitres=0;
    uint16_t reading1,reading2;
    uint16_t min1,min2;
    
    //SPIKE_PIN is GPIO3 = RX0 because hardcoded in i2s - remove UART cable from RX0 port!
    i2s_pins_t i2s_pins = {.data = true, .clock = false, .ws = false};
    i2s_clock_div_t clock_div = i2s_get_clock_div(53333333); // 53MHz provides div=3 and 0.01875 microseconds per step
    i2s_dma_init(NULL, NULL, clock_div, i2s_pins);
    dma_block.owner = 1; dma_block.sub_sof = 0; dma_block.unused = 0;
    dma_block.next_link_ptr = 0; dma_block.eof = 1; //only one block with EOF so will be a single shot
    dma_block.datalen = BUFSIZE*4; dma_block.blocksize = BUFSIZE*4; // uint32_t is 4byte data x BUFSIZE words
    dma_block.buf_ptr = dma_buf;
    dma_buf[i++]=ONES; dma_buf[i++]=ONES;dma_buf[i++]=~(ONES>>19); //83 bits (32+32+19) gets us 321 kHz
    dma_buf[i++]=ZEROS;dma_buf[i++]=ZEROS;dma_buf[i++]=ONES>>6; // we generate 3 pulses of 1.56 microseconds
    dma_buf[i++]=ONES;dma_buf[i++]=~(ONES>>25);dma_buf[i++]=ZEROS;dma_buf[i++]=ZEROS;
    dma_buf[i++]=ONES>>12;dma_buf[i++]=ONES;dma_buf[i++]=~(ONES>>31);
    dma_buf[i++]=ZEROS;dma_buf[i++]=ZEROS;dma_buf[i++]=ZEROS; //need to fill multiple of 4 32 bits words, so 16
    while (1) { //because GPIO3=I2S output is LOW in rest between shots, we must generate a HIGH pulse.
        min1=1023;min2=1023;
        for (i=0;i<WINDOW;i++) {
            gpio_write(COIL1_PIN, 0); //enable COIL1
            sdk_os_delay_us(20); //stabilise the output?
            i2s_dma_start(&dma_block); //transmit the dma_buf once
            reading1=sdk_system_adc_read();
            sdk_os_delay_us(200); //stabilise the output?
            gpio_write(COIL1_PIN, 1); //disable COIL1
            if (min1>reading1) min1=reading1;
            vTaskDelay(1); // 10ms
    
            gpio_write(COIL2_PIN, 0); //enable COIL2
            sdk_os_delay_us(20); //stabilise the output?
            i2s_dma_start(&dma_block); //transmit the dma_buf once
            reading2=sdk_system_adc_read();
            sdk_os_delay_us(200); //stabilise the output?
            gpio_write(COIL2_PIN, 1); //disable COIL2
            if (min2>reading2) min2=reading2;
            vTaskDelay(1); // 10ms
        }
        if (direction) {
            if ((min1-min2)>OFFSET+HYSTERESIS) {
                halflitres++;
                direction=0;
                ts = time(NULL);
                printf("%3.1f litres at %s",halflitres/2.0,ctime(&ts));
            }
        } else {
            if ((min1-min2)<OFFSET-HYSTERESIS) {
                halflitres++;
                direction=1;
                ts = time(NULL);
                printf("%3.1f litres at %s",halflitres/2.0,ctime(&ts));
            }
        }
        printf("%d %6d %3d %3d %3.1f\n",direction,sdk_system_get_time()/1000,min1,min2,halflitres/2.0);
    }
}

void sntp_task(void *argv) {
    setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1); tzset();
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "0.pool.ntp.org");
    sntp_setservername(1, "1.pool.ntp.org");
    sntp_setservername(2, "2.pool.ntp.org");
    while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) vTaskDelay(200/portTICK_PERIOD_MS); //Check if we have an IP every 200ms
    sntp_init();
    do {ts = time(NULL);
        if (ts == ((time_t)-1)) printf("ts=-1 ");
        vTaskDelay(100/portTICK_PERIOD_MS);
    } while (!(ts>1634567890)); //Mon Oct 18 16:38:10 CEST 2021
    printf("TIME SET: %u=%s", (unsigned int) ts, ctime(&ts));
    while(1) {
        vTaskDelay(6000); //60s
        ts = time(NULL);
        printf("TIME: %s", ctime(&ts));
    }
    vTaskDelete(NULL);
}


void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(2);
    UDPLUS("\n\n\nWaterMeter " VERSION "\n");
    gpio_enable( COIL1_PIN, GPIO_OUTPUT); gpio_write( COIL1_PIN, 1);
    gpio_enable( COIL2_PIN, GPIO_OUTPUT); gpio_write( COIL2_PIN, 1);
    xTaskCreate(spike_task, "Spike", 512, NULL, 3, NULL);
    xTaskCreate( sntp_task, "SNTP" , 512, NULL, 1, NULL);
}
