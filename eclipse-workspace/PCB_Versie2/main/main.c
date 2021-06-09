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
#include "IMU_Sensor.h"
#include "Vibratormotor.h"
#include "Touchpad.h"
#include "Resistive_Stretch.h"
#include "Speaker.h"
#include "Capacitive_Stretch.h"
#include <math.h>

#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include <sys/unistd.h>
#include <sys/stat.h>

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
#define PIN_NUM_CS   4

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

volatile int Tijd1, Tijd2, Tijd3, Tijd4;
volatile int CapWaarde1, CapWaarde2;
volatile int ResWaarde1, ResWaarde2;

int64_t get_current_time()
{
	int64_t temp;
	int64_t microseconden;

	temp = esp_timer_get_time();
	microseconden = temp - starttime;

	return microseconden;
}

void IRAM_ATTR power_off_isr_handler(void *arg)
{
	if(get_current_time() > 2500000)
	{
		gpio_set_level(power_on_off_pin, 0);
	}
}

void IRAM_ATTR CapStretch_ISR_Handler(void *arg)
{
	uint64_t counter;
	counter = get_counter(&counter);
	uint64_t TempVal = counter;
	PeriodCount = TempVal - StartValue;
	StartValue = TempVal;
}

static void Cap_NE555_Task(void *pvParameter)
{
	while(1)
	{
		Tijd1 = get_current_time();
		CapWaarde1 = PeriodCount;
		//printf("Capacitieve stretch sensor: ""%" PRId64 "," "%" PRId64 "\n", microseconden, Value);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


static void Interrupt_init(void)
{
	gpio_set_intr_type(power_sense_pin, GPIO_INTR_POSEDGE);
	gpio_set_intr_type(capacitive_stretch_pin, GPIO_INTR_NEGEDGE);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(power_sense_pin, power_off_isr_handler, NULL);
	gpio_isr_handler_add(capacitive_stretch_pin, CapStretch_ISR_Handler, NULL);
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


// ************************ //
// TCP Client initialisatie //
// ************************ //
static void tcp_client_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
}

// ************** //
// Trilmotor taak //
// ************** //

spi_device_handle_t SPI_init(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=2000000,           		//Clock out at 1 MHz
        .mode=0,                                //SPI mode 3
        .spics_io_num=-1,               		//CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(IMU_HOST, &buscfg, 2);
    //ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    //ESP_ERROR_CHECK(ret);
    imu_cs_init();
    return spi;
}


static void Speaker_task(void *pvParameter)
{
	while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
		speaker_on(1000, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		speaker_on(2000, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		speaker_on(3000, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		speaker_on(4000, 1);
		vTaskDelay(100 / portTICK_PERIOD_MS);
		speaker_on(1000, 0);
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

static void Motor1_task(void *pvParameter)
{
	while(1)
	{
		vibratormotor_on(0);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		vibratormotor_on(128);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    static char a[1024];
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
        	snprintf(a, sizeof a, "%d,%d,%d,%d,%d,%d,%d,%d", Tijd1, CapWaarde1, Tijd2, CapWaarde2, Tijd3, ResWaarde1, Tijd4, ResWaarde2);
        	//printf("STUCK?\n");
            int err = send(sock, test, strlen(test), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(100 / portTICK_PERIOD_MS);
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

static void Resistive_stretch1_task(void *pvParameter)
{
	int64_t microseconden = 0;
	float sensor_resistance;
	while(1)
	{
		ResWaarde1 = measure_resistance(adc_characteristics, adc_resistive_stretch1_channel);
		Tijd3 = get_current_time();
        //printf("%" PRId64 ", %0.1fOhm\n", microseconden, sensor_resistance);
        vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

static void Resistive_stretch2_task(void *pvParameter)
{
	int64_t microseconden = 0;
	float sensor_resistance;

	while(1)
	{
		ResWaarde2 = measure_resistance(adc_characteristics, adc_resistive_stretch2_channel);
		Tijd4 = get_current_time();
       // printf("%" PRId64 ", %0.1fOhm\n", microseconden, sensor_resistance);
        vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

static void Battery_task(void *pvParameter)
{
	int battery_voltage = 0;
	while(1)
	{
		battery_voltage = measure_battery_voltage(adc_characteristics, adc_bms_channel);
		control_leds(battery_voltage);
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

static void IMU_task(void *pvParameter)
{
	int64_t temp = 0;
	int64_t temp2 = 0;
	spi_device_handle_t spi = *(spi_device_handle_t *)pvParameter;
	while(1)
	{
		temp = esp_timer_get_time();
		IMU_read_data(spi);
	    temp2 = esp_timer_get_time();
	    temp2 = temp2-temp;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static void touchpad_task(void *pvParameter)
{
    int64_t microseconden;
    uint16_t touch_value;
    while (1) {
    	Tijd2 = get_current_time();
    	CapWaarde2 = read_touchpad();
        //printf("%" PRId64 ", %4d\n", microseconden, touch_value);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
	starttime = esp_timer_get_time();
	capacitive_stretch_init();
	vibrator_motor_init();
	//speaker_init();
	gpio_bms_init();
    ADC_init();
    Interrupt_init();
    touchpad_init();

//	static spi_device_handle_t spi;
//	spi = SPI_init();
//	IMU_write_reg(spi, 0x06, 0x01);
//	IMU_read_ID(spi);
//	IMU_init_Magneto(spi);
//
    tcp_client_init();
	xTaskCreate(&Battery_task, "Battery voltage measurement", 2048, NULL, 1, NULL);
//	xTaskCreate(&Motor1_task, "Vibrator Motor 1", 2048, NULL, 2, NULL);
//	xTaskCreate(&Speaker_task, "Speaker", 2048, NULL, 2, NULL);
    xTaskCreate(&Resistive_stretch1_task, "Resistive stretch sensor 1", 2048, NULL, 4, NULL);
    xTaskCreate(&Resistive_stretch2_task, "Resistive stretch sensor 2", 2048, NULL, 4, NULL);
	xTaskCreate(&tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    xTaskCreate(&touchpad_task, "Touchpad", 2048, NULL, 3, NULL);
	xTaskCreate(&Cap_NE555_Task, "Capacitive Stretch Sensor NE555", 2048, NULL,  4, NULL);
//	xTaskCreate(&IMU_task, "IMU", 2048, &spi, 4, NULL);
}
