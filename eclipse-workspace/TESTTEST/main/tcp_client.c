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

static void IRAM_ATTR pcnt_isr(void *para)
{
	//timer_pause();
	//aa++;
	uint64_t timer_counter_value;
	timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_counter_value);
	aa = timer_counter_value;
	//printf("%" PRIu64 "\n", timer_counter_value);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
	//timer_start();
	//portYIELD_FROM_ISR();
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

void IRAM_ATTR timer_group0_isr(void *para)
{
	int16_t blabla;
	timer_spinlock_take(TIMER_GROUP_0);
    //uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
    pcnt_get_counter_value(PCNT_UNIT_0, &blabla);
    aa = blabla;
    pcnt_counter_clear(PCNT_UNIT_0);

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, 4000000);

    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_spinlock_give(TIMER_GROUP_0);
}

void IRAM_ATTR power_off_isr_handler(void *arg)
{
	if(get_current_time() > 2500000)
	{
		close_FLAG = 1;
	}
}

static void IRAM_ATTR CapStretch_ISR_Handler(void *arg)
{
	uint64_t counter;
	timer_get_counter_value(TIMER_GROUP_1, TIMER_1, &counter);
	PeriodCount = counter - StartValue;
	StartValue = counter;
}

static void Cap_NE555_Task(void *pvParameter)
{
	int i = 0;
	int buffercap[10];
	int64_t buffertime[10];
	float waarde;
	while(1)
	{
		buffertime[i] = get_current_time();
		buffercap[i] = aa;
		waarde = buffercap[i];
		waarde = 1/(waarde/40000000/203);
		waarde = (1.44/waarde)/450*1000*1000*1000;
		waarde = (waarde-1167.5)/197.5*1000+10000;
		buffercap[i] = waarde;
		printf("sensor 1: %d\n", buffercap[i]);
		i++;
		if(i == 10)
		{
			for(int a = 0; a < 10; a++)
			{
				Cap1Buffer[a] = buffercap[a];
				Time1Buffer[a] = buffertime[a];
			}
			write_cap1_FLAG = 1;
			wifi_write_cap1_FLAG = 1;
			i = 0;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
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
		printf("sensor 2: %d\n", buffercap[i]);
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
//	gpio_set_intr_type(capacitive_stretch_pin, GPIO_INTR_NEGEDGE);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(power_sense_pin, power_off_isr_handler, NULL);
//	gpio_isr_handler_add(capacitive_stretch_pin, CapStretch_ISR_Handler, NULL);
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
//    		printf("In while loop\n");
    		if((wifi_write_cap1_FLAG == 1) && (wifi_write_imu_FLAG == 1) && (wifi_write_cap2_FLAG == 1) && (wifi_write_res1_FLAG == 1) && (wifi_write_res2_FLAG == 1))
//    		if((wifi_write_cap1_FLAG == 1) &&  (wifi_write_cap2_FLAG == 1))
    		{
//    			printf("in if statement\n");
    			wifi_write_cap1_FLAG = 0;
    			wifi_write_cap2_FLAG = 0;
    			wifi_write_res1_FLAG = 0;
    			wifi_write_res2_FLAG = 0;
    			wifi_write_imu_FLAG = 0;


    			int ok = 0;
//    			int c = 0;
//    			printf("Begin write\n");
    			for(int b = 0; b < 10; b++)
    			{
    				for(int c = (0+ok); c < (ok+10); c++)
    				{
    					snprintf(a, sizeof a, "%" PRId64 ",%d,"
    							"%" PRId64 ",%d,"
								"%" PRId64 ",%d,"
								"%" PRId64 ",%d,"
								"%" PRId64 ",%d,%d,%d,%d,%d,%d",
    							Time1Buffer[b], Cap1Buffer[b], Time2Buffer[b], Cap2Buffer[b],
    							Time3Buffer[b], Res1Buffer[b], Time4Buffer[b], Res2Buffer[b],
    							Time5Buffer[c], AcceleroX[c], AcceleroY[c], AcceleroZ[c],
    							GyroX[c], GyroY[c], GyroZ[c]);
    					//fprintf(f, test);

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
    	                }


//    					printf("b = %d\t c = %d\n", b, c);
    				}
    				ok = ok + 10;
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

static void Resistive_stretch1_task(void *pvParameter)
{
	int i = 0;
	int bufferres[10];
	int64_t buffertime[10];
	float waarde = 0;

	while(1)
	{
		ResWaarde1 = measure_resistance(adc_characteristics, adc_resistive_stretch1_channel);
		buffertime[i] = get_current_time();
		waarde = (ResWaarde1-1683.3)/75*1000+16000;
		bufferres[i] = waarde;
		printf("%d\n", bufferres[i]);
		i++;
		if(i == 10)
		{
			for(int a = 0; a < 10; a++)
			{
				Res1Buffer[a] = bufferres[a];
				Time3Buffer[a] = buffertime[a];
			}
			write_res1_FLAG = 1;
			wifi_write_res1_FLAG = 1;
			i = 0;
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

static void Resistive_stretch2_task(void *pvParameter)
{
	int i = 0;
	int bufferres[10];
	int64_t buffertime[10];
	float waarde = 0;

	while(1)
	{
		ResWaarde2 = measure_resistance(adc_characteristics, adc_resistive_stretch2_channel);
		buffertime[i] = get_current_time();
		waarde = (ResWaarde1-1683.3)/75*1000+16000;
		bufferres[i] = waarde;
		printf("%d\n", bufferres[i]);
		i++;
		if(i == 10)
		{
			for(int a = 0; a < 10; a++)
			{
				Res2Buffer[a] = bufferres[a];
				Time4Buffer[a] = buffertime[a];
			}
			write_res2_FLAG = 1;
			wifi_write_res2_FLAG = 1;
			i = 0;
		}
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
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void IMU_task(void *pvParameter)
{
	spi_device_handle_t spi = *(spi_device_handle_t *)pvParameter;

	int i = 0;
	int buffer_accelero_x[100];
	int buffer_accelero_y[100];
	int buffer_accelero_z[100];
	int buffer_gyro_x[100];
	int buffer_gyro_y[100];
	int buffer_gyro_z[100];
	int64_t buffer_time[100];
	int accelero_x, accelero_y, accelero_z;
	int gyro_x, gyro_y, gyro_z;
	int magn_x, magn_y, magn_z;
	while(1)
	{
		IMU_read_data(spi, &accelero_x, &accelero_y, &accelero_z, &gyro_x, &gyro_y, &gyro_z, &magn_x, &magn_y, &magn_z);
		printf("Accel XYZ (mg) = [%d, %d, %d]\t"
				"Gyro XYZ (dps) = [%d, %d, %d]\t"
				"Magn XYZ (uT) = [%d, %d, %d]\n"
				,accelero_x, accelero_y, accelero_z
				,gyro_x, gyro_y, gyro_z
				,magn_x, magn_y, magn_z);

		buffer_time[i] = get_current_time();
		buffer_accelero_x[i] = accelero_x;
		buffer_accelero_y[i] = accelero_y;
		buffer_accelero_z[i] = accelero_z;
		buffer_gyro_x[i] = gyro_x;
		buffer_gyro_y[i] = gyro_y;
		buffer_gyro_z[i] = gyro_z;

		i++;

		if(i == 100)
		{
			for(int a = 0; a < 100; a++)
			{
				AcceleroX[a] = buffer_accelero_x[a];
				AcceleroY[a] = buffer_accelero_y[a];
				AcceleroZ[a] = buffer_accelero_z[a];
				GyroX[a] = buffer_gyro_x[a];
				GyroY[a] = buffer_gyro_y[a];
				GyroZ[a] = buffer_gyro_z[a];
				Time5Buffer[a] = buffer_time[a];
			}
			write_imu_FLAG = 1;
			wifi_write_imu_FLAG = 1;
			i = 0;
		}
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

static void sd(void *pvParameter)
{
    static char a[1024];
    static char *test = &a;
    static char d[1024];
    static char *SDname = &d;
    int abc = 0;
    int bac = 1;
    int nametakenFLAG = 0;
    int filenummer = 0;

		ESP_LOGI(TAG, "Opening file");
		f = fopen(MOUNT_POINT"/hello.txt", "w");
		if (f == NULL) {
			ESP_LOGE(TAG, "Failed to open file for writing");
			return;
		}
	while(1)
	{
		printf("In while loop\n");
		if((write_cap1_FLAG == 1) && (write_imu_FLAG == 1) && (write_cap2_FLAG == 1) && (write_res1_FLAG == 1) && (write_res2_FLAG == 1))
		{
			printf("in if statement\n");
			write_cap1_FLAG = 0;
			write_cap2_FLAG = 0;
			write_res1_FLAG = 0;
			write_res2_FLAG = 0;
			write_imu_FLAG = 0;


			int ok = 0;
			printf("Begin write\n");
			for(int b = 0; b < 10; b++)
			{
				for(int c = (0+ok); c < (ok+10); c++)
				{
					snprintf(a, sizeof a, "%" PRId64 ",%d," "%" PRId64 ",%d," "%" PRId64 ",%d," "%" PRId64 ",%d," "%" PRId64 ",%d,%d,%d,%d,%d,%d\n",
							Time1Buffer[b], Cap1Buffer[b], Time2Buffer[b], Cap2Buffer[b],
							Time3Buffer[b], Res1Buffer[b], Time4Buffer[b], Res2Buffer[b],
							Time5Buffer[c], AcceleroX[c], AcceleroY[c], AcceleroZ[c],
							GyroX[c], GyroY[c], GyroZ[c]);
					fprintf(f, test);
					//printf("b = %d\t c = %d\n", b, c);
				}
				ok = ok + 10;
			}
			printf("End write\n");
		}
		if(close_FLAG == 1)
		{
			fclose(f);
			ESP_LOGI(TAG, "File written");

			// Check if destination file exists before renaming
			struct stat st;
			if (stat(MOUNT_POINT"/foo.txt", &st) == 0) {
				// Delete it if it exists
				unlink(MOUNT_POINT"/foo.txt");
			}

			// Rename original file
			ESP_LOGI(TAG, "Renaming file");
			if (rename(MOUNT_POINT"/hello.txt", MOUNT_POINT"/foo.txt") != 0) {
				ESP_LOGE(TAG, "Rename failed");
				return;
			}

			gpio_set_level(power_on_off_pin, 0);
		}
		vTaskDelay(200 / portTICK_PERIOD_MS);
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
	pcnt_config_t pcnt_config = {
	        .pulse_gpio_num = 27,
	        .ctrl_gpio_num = -1,
	        .lctrl_mode = PCNT_MODE_KEEP,
	        .hctrl_mode = PCNT_MODE_KEEP,
	        .pos_mode = PCNT_COUNT_INC,
			.neg_mode = PCNT_COUNT_DIS,
			.counter_h_lim = 200,
			.counter_l_lim = 0,
			.unit = PCNT_UNIT_0,
			.channel = PCNT_CHANNEL_0
	    };

	pcnt_unit_config(&pcnt_config);

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

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_1, PCNT_EVT_H_LIM);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_isr, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_isr_handler_add(PCNT_UNIT_1, pcnt_isr2, NULL);
    pcnt_intr_enable(PCNT_UNIT_1);

        /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_1);
}

void app_main(void)
{
	starttime = esp_timer_get_time();
	//capacitive_stretch_init();
	vibrator_motor_init();
	speaker_init();
    example_tg0_timer_init();
    pcnt_init();
	gpio_bms_init();
    ADC_init();
    Interrupt_init();
    touchpad_init();

	esp_err_t ret;
	// Options for mounting the filesystem.
	// If format_if_mount_failed is set to true, SD card will be partitioned and
	// formatted in case when mounting fails.
	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
	#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
		.format_if_mount_failed = true,
	#else
		.format_if_mount_failed = false,
	#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
		.max_files = 5,
		.allocation_unit_size = 16 * 1024
	};
	sdmmc_card_t* card;
	const char mount_point[] = MOUNT_POINT;
	ESP_LOGI(TAG, "Initializing SD card");


	ESP_LOGI(TAG, "Using SPI peripheral");

	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	spi_bus_config_t bus_cfg = {
		.mosi_io_num = SD_NUM_MOSI,
		.miso_io_num = SD_NUM_MISO,
		.sclk_io_num = SD_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4000,
	};
	ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to initialize bus.");
		return;
	}

	// This initializes the slot without card detect (CD) and write protect (WP) signals.
	// Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
	sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
	slot_config.gpio_cs = SD_NUM_CS;
	slot_config.host_id = host.slot;

	ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount filesystem. "
				"If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
		} else {
			ESP_LOGE(TAG, "Failed to initialize the card (%s). "
				"Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
		}
		return;
	}

	// Card has been initialized, print its properties
	sdmmc_card_print_info(stdout, card);

	static spi_device_handle_t spi;
	spi = SPI_init();
	IMU_write_reg(spi, 0x06, 0x01);
	IMU_read_ID(spi);
	IMU_init_Magneto(spi);
//
	xTaskCreate(&Battery_task, "Battery voltage measurement", 8192, NULL, 1, NULL);
	xTaskCreate(&Motor1_task, "Vibrator Motor 1", 2048, NULL, 2, NULL);
	xTaskCreate(&Speaker_task, "Speaker", 2048, NULL, 2, NULL);
	xTaskCreate(&sd, "sdcard", 16384, NULL,  3, NULL);
    xTaskCreate(&Resistive_stretch1_task, "Resistive stretch sensor 1", 8192, NULL, 5, NULL);
    xTaskCreate(&Resistive_stretch2_task, "Resistive stretch sensor 2", 8192, NULL, 5, NULL);
	xTaskCreate(&Cap_NE555_Task, "Capacitive Stretch Sensor NE555", 8192, NULL,  5, NULL);
	xTaskCreate(&Cap_NE555_Task2, "Capacitive Stretch Sensor2 NE555", 8192, NULL,  5, NULL);
	xTaskCreate(&IMU_task, "IMU", 65536, &spi, 4, NULL);
	tcp_client_init();
	xTaskCreate(&tcp_client_task, "tcp_client", 8192, NULL, 3, NULL);
    xTaskCreate(&touchpad_task, "Touchpad", 2048, NULL, 3, NULL);
}
