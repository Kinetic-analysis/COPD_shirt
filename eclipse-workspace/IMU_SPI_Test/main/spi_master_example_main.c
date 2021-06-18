/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "pretty_effect.h"
#include "IMU_sensor.h"
#include "BMS.h"
#include "driver/timer.h"
#include <inttypes.h>

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:

#define USE_SPI_MODE

// ESP32-S2 doesn't have an SD Host peripheral, always use SPI:
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

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.


// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define SD_CS   4



#define DEFAULT_VREF    	3300
#define NO_OF_SAMPLES   	16

static esp_adc_cal_characteristics_t *adc_characteristics;
static const adc_channel_t adc_resistive_stretch1_channel = ADC_CHANNEL_0;
static const adc_channel_t adc_resistive_stretch2_channel = ADC_CHANNEL_3;
static const adc_channel_t adc_bms_channel = ADC_CHANNEL_7;
static const adc_bits_width_t adc_resolution = ADC_WIDTH_BIT_12;
static const adc_atten_t adc_attenuation = ADC_ATTEN_DB_11;
static const adc_unit_t adc_unit = ADC_UNIT_1;

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/


//#define IMU_HOST    VSPI_HOST
//#define DMA_CHAN    2
//
//#define PIN_NUM_MISO 19
//#define PIN_NUM_MOSI 23
//#define PIN_NUM_CLK  18
//#define PIN_NUM_CS   5

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
//#define PARALLEL_LINES 16

volatile int64_t starttime;
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

static void Battery_task(void *pvParameter)
{
	int voltage = 0;
	while(1)
	{
		voltage = measure_battery_voltage(adc_characteristics, adc_bms_channel);
		control_leds(voltage);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

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
        .clock_speed_hz=400000,           		//Clock out at 1 MHz
        .mode=0,                                //SPI mode 3
        .spics_io_num=-1,               		//CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        //.pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(IMU_HOST, &buscfg, 2);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    return spi;
}

static void IMU_task(void *pvParameter)
{
	spi_device_handle_t spi = *(spi_device_handle_t *)pvParameter;
    gpio_set_level(PIN_NUM_CS, 1);				//CS = Hoog
	while(1)
	{
		IMU_read_data(spi);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void SD_Card_init(void)
{
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t* card;
    const char mount_point[] = MOUNT_POINT;
//    ESP_LOGI(TAG, "Initializing SD card");
//
//
//    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
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
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;
    ESP_LOGI(TAG, "Before");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    ESP_LOGI(TAG, "After");
//    if (ret != ESP_OK) {
//        if (ret == ESP_FAIL) {
//            ESP_LOGE(TAG, "Failed to mount filesystem. "
//                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
//        } else {
//            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
//                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
//        }
//        return;
//    }

    // Card has been initialized, print its properties
    //sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen(MOUNT_POINT"/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    //fprintf(f, "Hello %s!\n", card->cid.name);
    fprintf(f, "Goeie dag!\n");
    fprintf(f, "Goeie dag numero twee!\n");
    fclose(f);
//    ESP_LOGI(TAG, "File written");

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(MOUNT_POINT"/foo.txt", &st) == 0) {
        // Delete it if it exists
        unlink(MOUNT_POINT"/foo.txt");
    }

    // Rename original file
//    ESP_LOGI(TAG, "Renaming file");
    if (rename(MOUNT_POINT"/hello.txt", MOUNT_POINT"/foo.txt") != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    // Open renamed file for reading
//    ESP_LOGI(TAG, "Reading file");
    f = fopen(MOUNT_POINT"/foo.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    char line2[64];
    fgets(line, sizeof(line), f);
    fgets(line2, sizeof(line2), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    char* pos2 = strchr(line2, '\n');
    if (pos2) {
        *pos2 = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line2);
    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
//    ESP_LOGI(TAG, "Card unmounted");
#ifdef USE_SPI_MODE
    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
#endif
}




void app_main(void)
{
	//starttime = esp_timer_get_time();
	int64_t temp = 0;
	int64_t temp2 = 0;
	gpio_bms_init();
	//Interrupt_init();
    //ADC_init();
    //temp = esp_timer_get_time();

    //temp2 = esp_timer_get_time();
    //temp2 = temp2-temp;
    //printf("%" PRId64 "\n", temp2);
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);				//CS = Hoog
	static spi_device_handle_t spi;
	//temp = esp_timer_get_time();
	spi = SPI_init();
	IMU_write_reg(spi, 0x06, 0x01); 				//Selecteerd de klok van de IMU
    IMU_read_ID(spi);							//Leest de ID van de IMU
    IMU_init_Magneto(spi);						//Initialiseert de magnetometer voor communicatie
    //xTaskCreate(&Battery_task, "Battery voltage measurement", 2048, NULL, 1, NULL);
    //xTaskCreate(&sd_task, "sdcard", 2048, NULL, 5, NULL);
    xTaskCreate(&IMU_task, "IMU", 2048, &spi, 5, NULL);
    //xTaskCreate(&IMU_task, "IMU", 2048, &spi, 5, NULL);
}












