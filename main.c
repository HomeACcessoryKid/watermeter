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

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>
#include <semphr.h>

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
#define  OFFSET -10       // how much coil1 is higher than coil2
#define  HYSTERESIS  17   // to prevent noise to trigger
uint32_t dma_buf[BUFSIZE];
static   dma_descriptor_t dma_block;
time_t ts;

SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
#define PUB_MSG_LEN 48  // suitable for a Domoticz counter update

uint32_t halflitres=0;

#define MQTT_PORT  1883
#define MQTT_HOST  "192.168.178.5"
#define MQTT_USER  "WaterMeter"
#define MQTT_PASS  "testingonly"
#define MQTT_topic "domoticz/in"
#define DMTCZ_idx  "62"
uint8_t mqtt_buf[100];  //global variable for debugging only 
mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

void spike_task(void *argv) {
    int i=0;
    bool direction=0;
    uint16_t reading,min1,min2,min1x,min2x,min1xx,min2xx,min1xxx,min2xxx; // x is for eXtreme which we will ignore
    char msg[PUB_MSG_LEN];
    
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
        if (!i) { // i will be WINDOW if no update
            snprintf(msg, PUB_MSG_LEN, "{\"idx\":" DMTCZ_idx ",\"nvalue\":0,\"svalue\":\"%.1f\"}", halflitres/2.0);
            if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) printf("Publish queue overflow.\n");
        }
        printf("%d %d %d %d %d %d %3.1f %d\n",direction,sdk_system_get_time()/1000,min1xx,min2xx,min1,min2,halflitres/2.0,min1-min2-OFFSET);
        for (i=0;i<20;i+=2) printf("%02x%02x ",mqtt_buf[i],mqtt_buf[i+1]); printf("| ");
        for (i=0;i<12;i+=2) printf("%02x%02x ",data.clientID.cstring[i],data.clientID.cstring[i+1]); printf("\n");
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

static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

#define BACKOFF1 100/portTICK_PERIOD_MS
static void  mqtt_task(void *pvParameters)
{
    int ret         = 0;
    int backoff = BACKOFF1;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_readbuf[100];

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    data.willFlag       = 0;
    data.MQTTVersion    = 3;
    data.clientID.cstring   = mqtt_client_id;
    data.username.cstring   = MQTT_USER;
    data.password.cstring   = MQTT_PASS;
    data.keepAliveInterval  = 10;
    data.cleansession   = 0;

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n", ret);
            vTaskDelay(backoff);
            if (backoff<BACKOFF1*128) backoff*=2; //max out at 12.8 seconds
            continue;
        }
        printf("done\n");
        backoff = BACKOFF1;
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100,
                      mqtt_readbuf, 100);

        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n", ret);
            mqtt_network_disconnect(&network);
            vTaskDelay(backoff);
            if (backoff<BACKOFF1*128) backoff*=2; //max out at 12.8 seconds
            continue;
        }
        printf("done\n");
        backoff = BACKOFF1;
        xQueueReset(publish_queue);

        while(1){

            char msg[PUB_MSG_LEN - 1] = "\0";
            while(xQueueReceive(publish_queue, (void *)msg, 0) ==
                  pdTRUE){
                printf("got message to publish\n");
                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, MQTT_topic , &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}

static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n", __func__, status );
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
                printf("WiFi: AP not found\n");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
                printf("WiFi: connection failed\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n");
        sdk_wifi_station_disconnect();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}

void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(2);
    UDPLUS("\n\n\nWaterMeter " VERSION "\n");
    gpio_enable( COIL1_PIN, GPIO_OUTPUT); gpio_write( COIL1_PIN, 1);
    gpio_enable( COIL2_PIN, GPIO_OUTPUT); gpio_write( COIL2_PIN, 1);
    xTaskCreate(spike_task, "Spike", 512, NULL, 3, NULL);
    xTaskCreate( sntp_task, "SNTP" , 512, NULL, 1, NULL);
    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 1, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 2, NULL);
}
