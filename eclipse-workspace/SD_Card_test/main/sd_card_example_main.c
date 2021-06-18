/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

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
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "driver/timer.h"
#include <inttypes.h>
#include "driver/spi_master.h"
#include <string.h>
#include <stdbool.h>

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
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define IMU_HOST    VSPI_HOST
#define PARALLEL_LINES 16

#define Power_sense_pin			GPIO_NUM_34
#define Power_on_off_pin		GPIO_NUM_33
#define Green_led_pin			GPIO_NUM_17
#define Yellow_led_pin			GPIO_NUM_16
#define Red_led_pin				GPIO_NUM_21
#define Vibrator_motor1_pin		GPIO_NUM_22
#define Vibrator_motor2_pin		GPIO_NUM_26
#define Touch_sensor			GPIO_NUM_2
#define Battery					GPIO_NUM_35
#define USE_SPI_MODE

static void GPIO_init(void)
{
	gpio_pad_select_gpio(Power_sense_pin);
	gpio_pad_select_gpio(Power_on_off_pin);
	gpio_pad_select_gpio(Green_led_pin);
	gpio_pad_select_gpio(Yellow_led_pin);
	gpio_pad_select_gpio(Red_led_pin);
	gpio_pad_select_gpio(Battery);

	gpio_set_direction(Power_sense_pin, GPIO_MODE_INPUT);
	gpio_set_direction(Power_on_off_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Green_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Yellow_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Red_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Battery, GPIO_MODE_INPUT);

	gpio_set_level(Green_led_pin, 1);
	gpio_set_level(Yellow_led_pin, 0);
	gpio_set_level(Red_led_pin, 0);
	gpio_set_level(Power_on_off_pin, 1); //Zet systeem meteen aan
}

static void sd(void *pvParameter)
{
	while(1)
	{
		ESP_LOGI(TAG, "Opening file");
		FILE* f = fopen(MOUNT_POINT"/hello.txt", "w");
		if (f == NULL) {
			ESP_LOGE(TAG, "Failed to open file for writing");
			return;
		}
		//fprintf(f, "Hello %s!\n", card->cid.name);
		fprintf(f, "Goeie dag!\n");
		fprintf(f, "Goeie dag ndxfcvvcxvcxvxcumero twee!\n");
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

		// Open renamed file for reading
		ESP_LOGI(TAG, "Reading file");
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
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

spi_device_handle_t SPI_init(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=19,
        .mosi_io_num=23,
        .sclk_io_num=18,
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
    gpio_set_direction(5, GPIO_MODE_OUTPUT);
    gpio_set_level(5, 1);				//CS = Hoog
    return spi;
}

void IMU_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data)
{
	spi_transaction_t t;
	uint8_t buf[2];

	gpio_set_level(5, 0);

	reg &= 0x7f;
	buf[0] = reg;
	buf[1] = data;
	memset(&t, 0, sizeof(t));
	t.length = 8*2;
	t.tx_buffer = buf;

	spi_device_transmit(spi, &t);
	gpio_set_level(5, 1);
}

void IMU_read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *data)
{
	spi_transaction_t t;
	gpio_set_level(5, 0);

	reg = ((reg & 0x7F) | 0x80);
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.tx_buffer = &reg;
	spi_device_transmit(spi, &t);

	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.flags = SPI_TRANS_USE_RXDATA;
	spi_device_transmit(spi, &t);

	gpio_set_level(5, 1);
	*data = t.rx_data[0];
}

void IMU_read_ID(spi_device_handle_t spi)
{
	//Leest de ID van de IMU sensor
	//De ID van de IMU is 0xAE
	uint8_t reg = 0x00;
	uint8_t id;

	uint8_t write;
	uint8_t read;

	//write REG_BANK_SEL
	reg = 0x7F;
	write = 0x00;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);

	reg = 0x00;
	IMU_read_reg(spi, reg, &id);
	printf("IMU ID: 0x%02X\n", id);
}

void app_main(void)
{
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
	slot_config.gpio_cs = PIN_NUM_CS;
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

	// Use POSIX and C standard library functions to work with files.
	// First create a file.
//	static spi_device_handle_t spi;
//	spi = SPI_init();
//	IMU_read_ID(spi);
	xTaskCreate(&sd, "sdcard", 2048, NULL,  5, NULL);
}
