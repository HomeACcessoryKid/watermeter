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
#include <espressif/esp8266/eagle_soc.h>
#include "mqtt-client.h"
#include <sysparam.h>
#include <adv_button.h>
#include <rboot-api.h>

#ifndef VERSION
 #error You must set VERSION=x.y.z to match github version tag x.y.z
#endif
#ifndef BUTTON_PIN
#error  BUTTON_PIN is not specified
#endif

#define  COIL1_PIN 4
#define  COIL2_PIN 5
//SPIKE_PIN is GPIO3 = RX0 because hardcoded in i2s
//DETECT_PIN is ADC
#define  ONES  0xffffffff // 32 bits of value one
#define  ZEROS 0x00000000 // 32 bits of value zero
#define  BUFSIZE 16       // must be multiple of 4
#define  WINDOW  20       // minimum value of # of samples
#define  OFFSET -10       // how much coil1 is higher than coil2
#define  HYSTERESIS  14   // to prevent noise to trigger
#define  RTC_ADDR    0x600013B0
#define  RTC_MAGIC   0xdecebeaa
uint32_t dma_buf[BUFSIZE];
static   dma_descriptor_t dma_block;
time_t ts;

char     *dmtczidx=NULL, *counter0=NULL;
uint32_t halflitres;
bool     direction=0;

void spike_task(void *argv) {
    int i=0;
    uint16_t reading,min1,min2,min1x,min2x,min1xx,min2xx,min1xxx,min2xxx; // x is for eXtreme which we will ignore
    
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
        min1=1024;min2=1024;min1x=1024;min2x=1024;min1xx=1024;min2xx=1024;min1xxx=1024;min2xxx=1024;
        for (i=0;i<WINDOW;i++) {
            gpio_write(COIL1_PIN, 0); //enable COIL1
            sdk_os_delay_us(20); //stabilise the output?
            i2s_dma_start(&dma_block); //transmit the dma_buf once
            reading=sdk_system_adc_read();
            sdk_os_delay_us(200); //stabilise the output?
            gpio_write(COIL1_PIN, 1); //disable COIL1
            if      (min1xxx>reading) {min1=min1x; min1x=min1xx; min1xx=min1xxx; min1xxx=reading;}
            else if (min1xx >reading) {min1=min1x; min1x=min1xx; min1xx=reading;}
            else if (min1x  >reading) {min1=min1x; min1x=reading;}
            else if (min1   >reading)  min1=reading;
            vTaskDelay(1); // 10ms
    
            gpio_write(COIL2_PIN, 0); //enable COIL2
            sdk_os_delay_us(20); //stabilise the output?
            i2s_dma_start(&dma_block); //transmit the dma_buf once
            reading=sdk_system_adc_read();
            sdk_os_delay_us(200); //stabilise the output?
            gpio_write(COIL2_PIN, 1); //disable COIL2
            if      (min2xxx>reading) {min2=min2x; min2x=min2xx; min2xx=min2xxx; min2xxx=reading;}
            else if (min2xx >reading) {min2=min2x; min2x=min2xx; min2xx=reading;}
            else if (min2x  >reading) {min2=min2x; min2x=reading;}
            else if (min2   >reading)  min2=reading;
            vTaskDelay(1); // 10ms
        }
        if (direction) {
            if ((min1-min2)>OFFSET+HYSTERESIS) {
                halflitres++;
                direction=0; i=0;
                ts = time(NULL);
                printf("%3.1f litres at %s",halflitres/2.0,ctime(&ts));
            }
        } else {
            if ((min1-min2)<OFFSET-HYSTERESIS) {
                halflitres++;
                direction=1; i=0;
                ts = time(NULL);
                printf("%3.1f litres at %s",halflitres/2.0,ctime(&ts));
            }
        }
        //save state to RTC memory
        WRITE_PERI_REG(RTC_ADDR+ 4,halflitres ); //uint32_t
        WRITE_PERI_REG(RTC_ADDR+ 8,direction  ); //boolean
        WRITE_PERI_REG(RTC_ADDR   ,RTC_MAGIC  );
        
        if (!i) { // i will be WINDOW if no update
            i=mqtt_client_publish("{\"idx\":%s,\"nvalue\":0,\"svalue\":\"%.1f\"}", dmtczidx, halflitres/2.0);
            if (i<0) printf("MQTT publish of counter failed because %s\n",MQTT_CLIENT_ERROR(i));
        }
        printf("%d %d %d %d %d %d %3.1f %d\n",direction,sdk_system_get_time()/1000,min1xx,min2xx,min1,min2,halflitres/2.0,min1-min2-OFFSET);
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
    uint32_t old_halflitres=0;
    while(1) {
        if (old_halflitres==halflitres) {
            char wm_zero[12]; sprintf(wm_zero,"%u",halflitres);
            sysparam_set_string("wm0",wm_zero); // sysparam does not actually write if value didn't change !
        } else {
            old_halflitres=halflitres;
        }
        vTaskDelay(6000); //60s
        ts = time(NULL);
        printf("TIME: %s", ctime(&ts));
    }
    vTaskDelete(NULL);
}

mqtt_config_t mqttconf=MQTT_DEFAULT_CONFIG;
char error[]="error";
void ota_string() {
    char *otas;
    if (sysparam_get_string("ota_string", &otas) == SYSPARAM_OK) {
        mqttconf.host=strtok(otas,";");
        mqttconf.user=strtok(NULL,";");
        mqttconf.pass=strtok(NULL,";");
        dmtczidx     =strtok(NULL,";");
        counter0     =strtok(NULL,";");
    }
    if (mqttconf.host==NULL) mqttconf.host=error;
    if (mqttconf.user==NULL) mqttconf.user=error;
    if (mqttconf.pass==NULL) mqttconf.pass=error;
    if (dmtczidx     ==NULL) dmtczidx     =error;
    if (counter0     ==NULL) counter0     =error;
}

void singlepress_callback(uint8_t gpio, void *args) {
    printf("single press = add 0.5 litres\n");
}

void doublepress_callback(uint8_t gpio, void *args) {
    printf("double press = remove 0.5 litres\n");
}

void longpress_callback(uint8_t gpio, void *args) {
    printf("long press = ota-update in 5 seconds\n");
    rboot_set_temp_rom(1); //select the OTA main routine
    sdk_system_restart();  //#include <rboot-api.h>
}

void device_init() {
    //sysparam_set_string("ota_string", "192.168.178.5;WaterMeter;testingonly;62;41532"); //can be used if not using LCM
    ota_string();
    char *wm0;
    halflitres=2*atof(counter0);
    if (sysparam_get_string("wm0", &wm0) == SYSPARAM_OK) halflitres=atoi(wm0);
	if (READ_PERI_REG(RTC_ADDR)==RTC_MAGIC) {
	    halflitres=READ_PERI_REG(RTC_ADDR+ 4);
        direction =READ_PERI_REG(RTC_ADDR+ 8);
    }
    mqtt_client_init(&mqttconf);
    int i=mqtt_client_publish("{\"idx\":%s,\"nvalue\":0,\"svalue\":\"%.1f\"}", dmtczidx, halflitres/2.0);
    if (i<0) printf("MQTT publish of counter failed because %s\n",MQTT_CLIENT_ERROR(i));
    
    gpio_enable( COIL1_PIN, GPIO_OUTPUT); gpio_write( COIL1_PIN, 1);
    gpio_enable( COIL2_PIN, GPIO_OUTPUT); gpio_write( COIL2_PIN, 1);

    adv_button_set_evaluate_delay(10);
    adv_button_create(BUTTON_PIN, true, false);
    adv_button_register_callback_fn(BUTTON_PIN, singlepress_callback, 1, NULL);
    adv_button_register_callback_fn(BUTTON_PIN, doublepress_callback, 2, NULL);
    adv_button_register_callback_fn(BUTTON_PIN, longpress_callback, 3, NULL);
}

void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(2);
    UDPLUS("\n\n\nWaterMeter " VERSION "\n");
    device_init();
    xTaskCreate(spike_task, "Spike", 512, NULL, 3, NULL);
    xTaskCreate( sntp_task, "SNTP" , 512, NULL, 1, NULL);
}
