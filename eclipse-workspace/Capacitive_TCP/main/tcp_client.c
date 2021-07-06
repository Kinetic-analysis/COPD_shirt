/* Touch Pad Read Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/timer.h"
#include <inttypes.h>
#include "driver/spi_master.h"
#include <string.h>
#include <stdbool.h>
#include "driver/touch_pad.h"
#include "BMS.h"
#include <math.h>

#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include <sys/unistd.h>
#include <sys/stat.h>

#include "driver/periph_ctrl.h"
#include "driver/pcnt.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

#ifdef CONFIG_IDF_TARGET_ESP32S2
#ifndef USE_SPI_MODE
#define USE_SPI_MODE
#endif // USE_SPI_MODE
// on ESP32-S2, DMA channel must be the same as host id
#define SPI_DMA_CHAN    host.slot
#endif //CONFIG_IDF_TARGET_ESP32S2

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN

#define USE_SPI_MODE

#define DEFAULT_VREF    	3300
#define NO_OF_SAMPLES   	16

#define IMU_HOST    VSPI_HOST
#define DMA_CHAN    2

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18

#define SD_NUM_MISO 12
#define SD_NUM_MOSI 13
#define SD_NUM_CLK  14
#define SD_NUM_CS   15

#define PARALLEL_LINES 16

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static esp_adc_cal_characteristics_t *adc_characteristics;
static const adc_channel_t adc_resistive_stretch1_channel = ADC_CHANNEL_0;
static const adc_channel_t adc_resistive_stretch2_channel = ADC_CHANNEL_3;
static const adc_channel_t adc_bms_channel = ADC_CHANNEL_7;
static const adc_bits_width_t adc_resolution = ADC_WIDTH_BIT_12;
static const adc_atten_t adc_attenuation = ADC_ATTEN_DB_11;
static const adc_unit_t adc_unit = ADC_UNIT_1;

volatile int64_t starttime;

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *payload = "Message from ESP32 ";

volatile int64_t start_measure_time;
volatile int start_measure = 0;

volatile uint64_t StartValue = 0;
volatile uint64_t PeriodCount = 0;

volatile int64_t Tijd1;
volatile int Tijd2, Tijd3, Tijd4;
volatile int CapWaarde1, CapWaarde2;
volatile int ResWaarde1, ResWaarde2;

volatile int Cap1Buffer[100];
volatile int Cap2Buffer[100];
volatile int Res1Buffer[100];
volatile int Res2Buffer[100];

volatile int64_t Time1Buffer[100];
volatile int64_t Time2Buffer[100];
volatile int64_t Time3Buffer[100];
volatile int64_t Time4Buffer[100];

volatile int AcceleroX[1000];
volatile int AcceleroY[1000];
volatile int AcceleroZ[1000];
volatile int GyroX[1000];
volatile int GyroY[1000];
volatile int GyroZ[1000];

volatile int64_t Time5Buffer[1000];

volatile int write_cap1_FLAG = 0;
volatile int write_cap2_FLAG = 0;
volatile int write_res1_FLAG = 0;
volatile int write_res2_FLAG = 0;
volatile int write_imu_FLAG = 0;

volatile int wifi_write_cap1_FLAG = 0;
volatile int wifi_write_cap2_FLAG = 0;
volatile int wifi_write_res1_FLAG = 0;
volatile int wifi_write_res2_FLAG = 0;
volatile int wifi_write_imu_FLAG = 0;


volatile int close_FLAG = 0;

volatile int aa = 0;
volatile int bb = 0;

volatile FILE* f;

int64_t get_current_time()
{
	int64_t temp;
	int64_t microseconden;

	temp = esp_timer_get_time();
	microseconden = temp - starttime;

	return microseconden;
}


static void IRAM_ATTR pcnt_isr2(void *para)
{
	//timer_pause();
	//aa++;
	uint64_t timer_counter_value;
	timer_get_counter_value(TIMER_GROUP_1, TIMER_1, &timer_counter_value);
	bb = timer_counter_value;
	//printf("%" PRIu64 "\n", timer_counter_value);
	timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);
	//timer_start();
	//portYIELD_FROM_ISR();
}

void IRAM_ATTR power_off_isr_handler(void *arg)
{
	if(get_current_time() > 2500000)
	{
		gpio_set_level(power_on_off_pin, 0);
	}
}


static void Cap_NE555_Task2(void *pvParameter)
{
	int i = 0;
	int buffercap[10];
	int64_t buffertime[10];
	float waarde;
	while(1)
	{
		buffertime[i] = get_current_time();
		buffercap[i] = bb;
		waarde = buffercap[i];
		waarde = 1/(waarde/40000000/203);
		waarde = (1.44/waarde)/450*1000*1000*1000;
		waarde = (waarde-1167.5)/197.5*1000+10000;
		buffercap[i] = waarde;

		i++;
		if(i == 10)
		{
			for(int a = 0; a < 10; a++)
			{
				Cap2Buffer[a] = buffercap[a];
				Time2Buffer[a] = buffertime[a];
			}
			write_cap2_FLAG = 1;
			wifi_write_cap2_FLAG = 1;
			i = 0;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


static void Interrupt_init(void)
{
	gpio_set_intr_type(power_sense_pin, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(power_sense_pin, power_off_isr_handler, NULL);
}

static void ADC_init(void)
{
	adc1_config_width(adc_resolution);
	adc1_config_channel_atten(adc_resistive_stretch1_channel, adc_attenuation);
	adc1_config_channel_atten(adc_resistive_stretch2_channel, adc_attenuation);
	adc1_config_channel_atten(adc_bms_channel, adc_attenuation);

	adc_characteristics = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(adc_unit, adc_attenuation, adc_resolution, DEFAULT_VREF, adc_characteristics);
}


static void tcp_client_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
}


static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    static char a[1023];
    static char *test = &a;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");
        vTaskDelay(500 / portTICK_PERIOD_MS);
        while (1) {
    		if((wifi_write_cap2_FLAG == 1))
    		{

    			wifi_write_cap2_FLAG = 0;


    			for(int b = 0; b < 10; b++)
    			{
    					snprintf(a, sizeof a, "%" PRId64 ",%d", Time2Buffer[b], Cap2Buffer[b]);

    	                int err = send(sock, test, strlen(test), 0);
    	                if (err < 0) {
    	                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    	                    break;
    	                }

    	                int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

    	                if (len < 0) {
    	                    ESP_LOGE(TAG, "recv failed: errno %d", errno);
    	                    break;
    	                }

    	                else {
    	                    rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
    	                }
    			}
    		}
            vTaskDelay(90 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
	while(1){
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}



static void Battery_task(void *pvParameter)
{
	int battery_voltage = 0;
	while(1)
	{
		battery_voltage = measure_battery_voltage(adc_characteristics, adc_bms_channel);
		control_leds(battery_voltage);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}



static void example_tg0_timer_init()
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = 2,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = 0,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_start(TIMER_GROUP_0, TIMER_0);

    /* Select and initialize basic parameters of the timer */
    timer_config_t config2 = {
        .divider = 2,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = 0,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_1, TIMER_1, &config2);
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL);
    timer_start(TIMER_GROUP_1, TIMER_1);
}


static void pcnt_init()
{
	pcnt_config_t pcnt_config2 = {
	        .pulse_gpio_num = 32,
	        .ctrl_gpio_num = -1,
	        .lctrl_mode = PCNT_MODE_KEEP,
	        .hctrl_mode = PCNT_MODE_KEEP,
	        .pos_mode = PCNT_COUNT_INC,
			.neg_mode = PCNT_COUNT_DIS,
			.counter_h_lim = 200,
			.counter_l_lim = 0,
			.unit = PCNT_UNIT_1,
			.channel = PCNT_CHANNEL_1
	    };

	pcnt_unit_config(&pcnt_config2);


//	pcnt_set_filter_value(PCNT_UNIT_0, 1023);
//	pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_isr2, NULL);
    pcnt_intr_enable(PCNT_UNIT_1);

    pcnt_counter_resume(PCNT_UNIT_1);
}

void app_main(void)
{
	starttime = esp_timer_get_time();
    example_tg0_timer_init();
    pcnt_init();
	gpio_bms_init();
    ADC_init();
    Interrupt_init();

	xTaskCreate(&Battery_task, "Battery voltage measurement", 8192, NULL, 1, NULL);
	xTaskCreate(&Cap_NE555_Task2, "Capacitive Stretch Sensor2 NE555", 8192, NULL,  5, NULL);
	tcp_client_init();
	xTaskCreate(&tcp_client_task, "tcp_client", 8192, NULL, 3, NULL);
}
