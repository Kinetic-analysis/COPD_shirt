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

int16_t TwosComplement_To_Integer(uint16_t integer)
{
	int16_t value = integer;
    if((integer >> 15) == 0b0000000000000001) //if its negative
    {
    	value = ~integer;
    	value = (integer + 1)*-1;
    }
    return value;
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

void IMU_read_gyro_registers(spi_device_handle_t spi)
{
	uint8_t reg;
	uint8_t read;
	uint8_t write;

	reg = 0x7F;
	write = 0x20;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);

	reg = 0x01;
	write = 0x01;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("GYRO_CONFIG_1: 0x%02X\n", read);

	reg = 0x7F;
	write = 0x00;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);
}

void IMU_read_ID2(spi_device_handle_t spi)
{
	uint8_t reg;
	uint8_t read;
	uint8_t write;

	//write REG_BANK_SEL
	reg = 0x7F;
	write = 0x00;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);

	//write USER_CTRL
	reg = 0x03;
	write = 0x20;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("USER_CTRL: 0x%02X\n", read);

	//write REG_BANK_SEL
	reg = 0x7F;
	write = 0x30;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);

	//write I2C_MST_CTRL
	reg = 0x01;
	write = 0x17;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_MST_CTRL: 0x%02X\n", read);

	//write I2C_MST_DELAY_CTRL
	reg = 0x02;
	write = 0x01;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_MST_DELAY_CTRL: 0x%02X\n", read);

	//write I2C_SLV0_ADDR
	reg = 0x03;
	write = 0x0C;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_SLV0_ADDR: 0x%02X\n", read);

	//write I2C_SLV0_REG
	reg = 0x04;
	write = 0x31;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_SLV0_REG: 0x%02X\n", read);

	//write I2C_SLV0_DO
	reg = 0x06;
	write = 0x08;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_SLV0_REG: 0x%02X\n", read);

	//write I2C_SLV0_CTRL
	reg = 0x05;
	write = 0x8A;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_SLV0_CTRL: 0x%02X\n", read);

	//write I2C_SLV0_ADDR
	reg = 0x03;
	write = 0x8C;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_SLV0_ADDR: 0x%02X\n", read);

	//write I2C_SLV0_REG
	reg = 0x04;
	write = 0x11;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("I2C_SLV0_REG: 0x%02X\n", read);

	//write REG_BANK_SEL
	reg = 0x7F;
	write = 0x00;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);
}

void IMU_read_data(spi_device_handle_t spi)
{

	// https://devzone.nordicsemi.com/f/nordic-q-a/36615/invensense-icm-20948

	uint8_t reg1;
	uint8_t reg2;
	uint8_t TEMP_OUT_H;
	uint8_t TEMP_OUT_L;
	uint8_t X_ACCEL_OUT_H;
	uint8_t X_ACCEL_OUT_L;
	uint8_t Y_ACCEL_OUT_H;
	uint8_t Y_ACCEL_OUT_L;
	uint8_t Z_ACCEL_OUT_H;
	uint8_t Z_ACCEL_OUT_L;
	uint8_t X_GYRO_OUT_H;
	uint8_t X_GYRO_OUT_L;
	uint8_t Y_GYRO_OUT_H;
	uint8_t Y_GYRO_OUT_L;
	uint8_t Z_GYRO_OUT_H;
	uint8_t Z_GYRO_OUT_L;

	int16_t TEMP_OUT;
	int16_t X_ACCEL_OUT;
	int16_t Y_ACCEL_OUT;
	int16_t Z_ACCEL_OUT;
	int16_t X_GYRO_OUT;
	int16_t Y_GYRO_OUT;
	int16_t Z_GYRO_OUT;


	float Temp_finalwaarde = 0;
	float X_ACCEL_finalwaarde = 0;
	float Y_ACCEL_finalwaarde = 0;
	float Z_ACCEL_finalwaarde = 0;
	float X_GYRO_finalwaarde;
	float Y_GYRO_finalwaarde;
	float Z_GYRO_finalwaarde;


	reg1 = 0x39;
	reg2 = 0x3A;
	IMU_read_reg(spi, reg1, &TEMP_OUT_H);
	IMU_read_reg(spi, reg2, &TEMP_OUT_L);
	TEMP_OUT = (TEMP_OUT_H << 8);
	TEMP_OUT |= TEMP_OUT_L;
	reg1 = 0x2D;
	reg2 = 0x2E;
	IMU_read_reg(spi, reg1, &X_ACCEL_OUT_H);
	IMU_read_reg(spi, reg2, &X_ACCEL_OUT_L);
	reg1 = 0x2F;
	reg2 = 0x30;
	IMU_read_reg(spi, reg1, &Y_ACCEL_OUT_H);
	IMU_read_reg(spi, reg2, &Y_ACCEL_OUT_L);
	reg1 = 0x31;
	reg2 = 0x32;
	IMU_read_reg(spi, reg1, &Z_ACCEL_OUT_H);
	IMU_read_reg(spi, reg2, &Z_ACCEL_OUT_L);
	reg1 = 0x33;
	reg2 = 0x34;
	IMU_read_reg(spi, reg1, &X_GYRO_OUT_H);
	IMU_read_reg(spi, reg2, &X_GYRO_OUT_L);
	reg1 = 0x35;
	reg2 = 0x36;
	IMU_read_reg(spi, reg1, &Y_GYRO_OUT_H);
	IMU_read_reg(spi, reg2, &Y_GYRO_OUT_L);
	reg1 = 0x37;
	reg2 = 0x38;
	IMU_read_reg(spi, reg1, &Z_GYRO_OUT_H);
	IMU_read_reg(spi, reg2, &Z_GYRO_OUT_L);

	TEMP_OUT = ((TEMP_OUT_H << 8) | (TEMP_OUT_L & 0xFF));
	X_ACCEL_OUT = ((X_ACCEL_OUT_H << 8) | (X_ACCEL_OUT_L & 0xFF));
	Y_ACCEL_OUT = ((Y_ACCEL_OUT_H << 8) | (Y_ACCEL_OUT_L & 0xFF));
	Z_ACCEL_OUT = ((Z_ACCEL_OUT_H << 8) | (Z_ACCEL_OUT_L & 0xFF));
	X_GYRO_OUT = ((X_GYRO_OUT_H << 8) | (X_GYRO_OUT_L & 0xFF));
	Y_GYRO_OUT = ((Y_GYRO_OUT_H << 8) | (Y_GYRO_OUT_L & 0xFF));
	Z_GYRO_OUT = ((Z_GYRO_OUT_H << 8) | (Z_GYRO_OUT_L & 0xFF));

	Temp_finalwaarde = ((TEMP_OUT-20)/333.87)+21;
	X_ACCEL_finalwaarde = X_ACCEL_OUT/16.384;
	Y_ACCEL_finalwaarde = Y_ACCEL_OUT/16.384;
	Z_ACCEL_finalwaarde = Z_ACCEL_OUT/16.384;
	X_GYRO_finalwaarde = X_GYRO_OUT/131;
	Y_GYRO_finalwaarde = Y_GYRO_OUT/131;
	Z_GYRO_finalwaarde = Z_GYRO_OUT/131;

	printf("Accel XYZ (mg) = [%0.2f, %0.2f, %0.2f]\t"
			"Gyro XYZ (dps) = [%0.2f, %0.2f, %0.2f]\t"
			"Temp (C) = [%0.2f]\n"
			,X_ACCEL_finalwaarde
			,Y_ACCEL_finalwaarde
			,Z_ACCEL_finalwaarde
			,X_GYRO_finalwaarde
			,Y_GYRO_finalwaarde
			,Z_GYRO_finalwaarde
			,Temp_finalwaarde);
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



void IMU_read_Magneto2(spi_device_handle_t spi)
{
	uint8_t reg;
	uint8_t data;
	uint8_t read;

	uint8_t X_MAGN_OUT_H;
	uint8_t X_MAGN_OUT_L;
	uint8_t Y_MAGN_OUT_H;
	uint8_t Y_MAGN_OUT_L;
	uint8_t Z_MAGN_OUT_H;
	uint8_t Z_MAGN_OUT_L;

	int16_t X_MAGN_OUT;
	int16_t Y_MAGN_OUT;
	int16_t Z_MAGN_OUT;

	float X_MAGN_finalwaarde;
	float Y_MAGN_finalwaarde;
	float Z_MAGN_finalwaarde;

	//read EXT_SLV_SENS_DATA_00
	reg = 0x3B;
	IMU_read_reg(spi, reg, &X_MAGN_OUT_L);
	//printf("EXT_SLV_SENS_DATA_00: 0x%02X\n", X_MAGN_OUT_L);
	reg = 0x3C;
	IMU_read_reg(spi, reg, &X_MAGN_OUT_H);
	//printf("EXT_SLV_SENS_DATA_01: 0x%02X\n", X_MAGN_OUT_H);
	reg = 0x3D;
	IMU_read_reg(spi, reg, &Y_MAGN_OUT_L);
	//printf("EXT_SLV_SENS_DATA_02: 0x%02X\n", Y_MAGN_OUT_L);
	reg = 0x3E;
	IMU_read_reg(spi, reg, &Y_MAGN_OUT_H);
	//printf("EXT_SLV_SENS_DATA_03: 0x%02X\n", Y_MAGN_OUT_H);
	reg = 0x3F;
	IMU_read_reg(spi, reg, &Z_MAGN_OUT_L);
	//printf("EXT_SLV_SENS_DATA_04: 0x%02X\n", Z_MAGN_OUT_L);
	reg = 0x40;
	IMU_read_reg(spi, reg, &Z_MAGN_OUT_H);
	//printf("EXT_SLV_SENS_DATA_05: 0x%02X\n", Z_MAGN_OUT_H);
	reg = 0x41;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_06: 0x%02X\n", read);
	reg = 0x42;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_07: 0x%02X\n", read);
	reg = 0x43;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_08: 0x%02X\n", read);
	reg = 0x44;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_09: 0x%02X\n", read);
	reg = 0x45;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_10: 0x%02X\n", read);
	reg = 0x46;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_11: 0x%02X\n", read);
	reg = 0x47;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_12: 0x%02X\n", read);
	reg = 0x48;
	IMU_read_reg(spi, reg, &read);
	//printf("EXT_SLV_SENS_DATA_13: 0x%02X\n", read);

	X_MAGN_OUT = ((X_MAGN_OUT_H << 8) | (X_MAGN_OUT_L & 0xFF));
	Y_MAGN_OUT = ((Y_MAGN_OUT_H << 8) | (Y_MAGN_OUT_L & 0xFF));
	Z_MAGN_OUT = ((Z_MAGN_OUT_H << 8) | (Z_MAGN_OUT_L & 0xFF));

	X_MAGN_finalwaarde = X_MAGN_OUT*0.15;
	Y_MAGN_finalwaarde = Y_MAGN_OUT*0.15;
	Z_MAGN_finalwaarde = Z_MAGN_OUT*0.15;

	printf("Magn XYZ (uT) = [%0.2f, %0.2f, %0.2f]\n"
			,X_MAGN_finalwaarde
			,Y_MAGN_finalwaarde
			,Z_MAGN_finalwaarde);
}


void app_main(void)
{
	uint8_t reg = 0x06;
	uint8_t data = 0x01;
	spi_device_handle_t spi;
	spi = SPI_init();
	IMU_write_reg(spi, reg, data);
    IMU_read_ID(spi);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    IMU_read_ID2(spi);
    while(1)
    {
    	//IMU_read_data(spi);
    	IMU_read_Magneto2(spi);
    	vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}












