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

#include "pretty_effect.h"

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/


#define IMU_HOST    VSPI_HOST
#define DMA_CHAN    2

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5


//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16



//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    //
}

void beginSPI(spi_device_handle_t spi)
{
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);		//CS = Hoog


}



bool IMU_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data)
{
	spi_transaction_t t;
	uint8_t buf[2];

	gpio_set_level(PIN_NUM_CS, 0);

	reg &= 0x7f;
	buf[0] = reg;
	buf[1] = data;
	memset(&t, 0, sizeof(t));
	t.length = 8*2;
	t.tx_buffer = buf;

	spi_device_transmit(spi, &t);
	gpio_set_level(PIN_NUM_CS, 1);
	return true;
}

bool IMU_read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *data)
{
	spi_transaction_t t;
	gpio_set_level(PIN_NUM_CS, 0);

	reg = ((reg & 0x7F) | 0x80);
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.tx_buffer = &reg;
	spi_device_transmit(spi, &t);

	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.flags = SPI_TRANS_USE_RXDATA;
	spi_device_transmit(spi, &t);

	gpio_set_level(PIN_NUM_CS, 1);
	*data = t.rx_data[0];

	return true;
}

void IMU_read_ID(spi_device_handle_t spi)
{
	uint8_t reg = 0x00;
	uint8_t id;

	IMU_read_reg(spi, reg, &id);
	printf("IMU ID: 0x%02X\n", id);
}

void IMU_read_Temp(spi_device_handle_t spi)
{
	uint8_t reg1 = 0x06;
	uint8_t TEMP_OUT_H;
	//uint8_t reg2 = 0x2E;
	//uint8_t TEMP_OUT_L;

	IMU_read_reg(spi, reg1, &TEMP_OUT_H);
	//IMU_read_reg(spi, reg2, &TEMP_OUT_L);
	printf("IMU Temp: 0x%02X\n", TEMP_OUT_H);
	//printf("IMU Temp: 0x%02X, 0x%02X\n", TEMP_OUT_H, TEMP_OUT_L);
}

void IMU_read_Temp2(spi_device_handle_t spi)
{
	uint8_t reg1 = 0x39;
	uint8_t TEMP_OUT_H;
	uint8_t reg2 = 0x3A;
	uint8_t TEMP_OUT_L;
	float finalwaarde;

	IMU_read_reg(spi, reg1, &TEMP_OUT_H);
	IMU_read_reg(spi, reg2, &TEMP_OUT_L);

	uint16_t TEMP_OUT = 0x0000;
	TEMP_OUT = (TEMP_OUT_H << 8);
	TEMP_OUT |= TEMP_OUT_L;
	printf("IMU Temp: 0x%02X, 0x%02X\n", TEMP_OUT_H, TEMP_OUT_L);
	printf("IMU Temp: 0x%04X\n", TEMP_OUT);
	finalwaarde = ((TEMP_OUT-20)/333.87)+21;
	printf("IMU Temp: %0.2f\n", finalwaarde);
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
        .clock_speed_hz=1000000,           //Clock out at 10 MHz
        .mode=0,                                //SPI mode 3
        .spics_io_num=-1,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(IMU_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);		//CS = Hoog
    return spi;
}

void app_main(void)
{
	uint8_t reg = 0x06;
	uint8_t data = 0x00;

	spi_device_handle_t spi;
	spi = SPI_init();
    IMU_read_ID(spi);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    IMU_write_reg(spi, reg, data);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while(1)
    {
    	IMU_read_ID(spi);
    	IMU_read_Temp(spi);
    	IMU_read_Temp2(spi);
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
