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

#define IMU_HOST    HSPI_HOST
#define DMA_CHAN    2

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

//To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many. More means more memory use,
//but less overhead for setting up / finishing transfers. Make sure 240 is dividable by this.
#define PARALLEL_LINES 16

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/

spi_device_handle_t SPI_init(void)
{
    gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_CS, 1);				//CS = Hoog

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
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    return spi;
}

void IMU_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data)
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
}

void IMU_read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *data)
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


void IMU_init_Magneto(spi_device_handle_t spi)
{
	//Initialiseert de magneto om te communiceren met de ICM-20498 via I2C
	//De magnetometer data wordt in de externe slave registers geplaatst
	uint8_t read[12];
	uint8_t reg[12] = {0x7F, 0x03, 0x7F, 0x01, 0x02, 0x03, 0x04, 0x06, 0x05, 0x03, 0x04, 0x7F};
	uint8_t write[12] = {0x00, 0x20, 0x30, 0x17, 0x01, 0x0C, 0x31, 0x08, 0x8A, 0x8C, 0x11, 0x00};

	for(int i = 0; i < 12; i++)
	{
		IMU_write_reg(spi, reg[i], write[i]);
		IMU_read_reg(spi, reg[i], &read[i]);
	}
}

void IMU_read_data(spi_device_handle_t spi)
{

	// https://devzone.nordicsemi.com/f/nordic-q-a/36615/invensense-icm-20948

	uint8_t reg[28] = {0x39, 0x3A, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
					   0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48};
	uint8_t data_HL[28];
	int16_t data_OUT[14];
	float data_Final[14];

	for(int i = 0; i < 28; i++)
	{
		IMU_read_reg(spi, reg[i], &data_HL[i]);	//Leest alle sensor registers uit
	}


	data_OUT[0] = ((data_HL[0] << 8) | (data_HL[1] & 0xFF));	//Combineert high en low byte van temperatuur sensor met two's complement
	data_OUT[1] = ((data_HL[2] << 8) | (data_HL[3] & 0xFF));	//Combineert high en low byte van accelerometer met two's complement
	data_OUT[2] = ((data_HL[4] << 8) | (data_HL[5] & 0xFF));	//Combineert high en low byte van accelerometer met two's complement
	data_OUT[3] = ((data_HL[6] << 8) | (data_HL[7] & 0xFF));	//Combineert high en low byte van accelerometer met two's complement
	data_OUT[4] = ((data_HL[8] << 8) | (data_HL[9] & 0xFF));	//Combineert high en low byte van gyroscope met two's complement
	data_OUT[5] = ((data_HL[10] << 8) | (data_HL[11] & 0xFF));	//Combineert high en low byte van gyroscope met two's complement
	data_OUT[6] = ((data_HL[12] << 8) | (data_HL[13] & 0xFF));	//Combineert high en low byte van gyroscope met two's complement

	data_OUT[7] = ((data_HL[15] << 8) | (data_HL[14] & 0xFF));	//Combineert high en low byte van Magnetometer X-as met Little Endian
	data_OUT[8] = ((data_HL[17] << 8) | (data_HL[16] & 0xFF));	//Combineert high en low byte van Magnetometer Y-as met Little Endian
	data_OUT[9] = ((data_HL[19] << 8) | (data_HL[18] & 0xFF));	//Combineert high en low byte van Magnetometer Z-as met Little Endian

	data_Final[0] = ((data_OUT[0]-20)/333.87)+21;	//Berekend de temperatuur in graden celcius
	data_Final[1] = data_OUT[1]/16.384;				//Berekend de accelerometer waarde van de X-axis in mg
	data_Final[2] = data_OUT[2]/16.384;				//Berekend de accelerometer waarde van de Y-axis in mg
	data_Final[3] = data_OUT[3]/16.384;				//Berekend de accelerometer waarde van de Z-axis in mg
	data_Final[4] = data_OUT[4]/131;				//Berekend de gyroscope waarde van de X-axis in dps
	data_Final[5] = data_OUT[5]/131;				//Berekend de gyroscope waarde van de Y-axis in dps
	data_Final[6] = data_OUT[6]/131;				//Berekend de gyroscope waarde van de Z-axis in dps
	data_Final[7] = data_OUT[7]*0.15;				//Berekend de magnetometer waarde van de X-axis in uT
	data_Final[8] = data_OUT[8]*0.15;				//Berekend de magnetometer waarde van de Y-axis in uT
	data_Final[9] = data_OUT[9]*0.15;				//Berekend de magnetometer waarde van de Z-axis in uT

	printf("Temp (C) = [%0.2f]\t "
			"Accel XYZ (mg) = [%0.2f, %0.2f, %0.2f]\t "
			"Gyro XYZ (dps) = [%0.2f, %0.2f, %0.2f]\t "
			"Magn XYZ (uT) = [%0.2f, %0.2f, %0.2f]\n"
			, data_Final[0],
			data_Final[1], data_Final[2], data_Final[3],
			data_Final[4], data_Final[5], data_Final[6],
			data_Final[7], data_Final[8], data_Final[9]);
}

void app_main(void)
{
	spi_device_handle_t spi;
	spi = SPI_init();
	IMU_write_reg(spi, 0x06, 0x01); 				//Selecteerd de klok van de IMU
    IMU_read_ID(spi);							//Leest de ID van de IMU
    IMU_init_Magneto(spi);						//Initialiseert de magnetometer voor communicatie
	IMU_read_data(spi);
	while(1)
	{
		IMU_read_data(spi);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}
