/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "IMU_Sensor.h"
#include "BMS.h"

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
        .clock_speed_hz=1000000,           		//Clock out at 1 MHz
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

static void IMU_task(void *pvParameter)
{
	spi_device_handle_t spi = *(spi_device_handle_t *)pvParameter;

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


		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void app_main(void)
{
	gpio_bms_init();

	static spi_device_handle_t spi;
	spi = SPI_init();
	IMU_write_reg(spi, 0x06, 0x01);
	IMU_read_ID(spi);
	IMU_init_Magneto(spi);



	xTaskCreate(&IMU_task, "IMU", 65536, &spi, 4, NULL);

}
