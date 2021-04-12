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

#define DEFAULT_VREF    	3300
#define NO_OF_SAMPLES   	16

#define uC_SENSE			GPIO_NUM_16
#define uC_LATCH			GPIO_NUM_17
#define GREEN_LED			GPIO_NUM_25
#define YELLOW_LED			GPIO_NUM_26
#define RED_LED				GPIO_NUM_27
#define VIBRATOR_MOTOR		GPIO_NUM_33

static esp_adc_cal_characteristics_t *ADC_CHARS;
static const adc_channel_t RESISTIVE_STRETCH = ADC_CHANNEL_6;
static const adc_channel_t BATT_VOLT = ADC_CHANNEL_7;
static const adc_bits_width_t WIDTH = ADC_WIDTH_BIT_12;
static const adc_atten_t ATTEN = ADC_ATTEN_DB_0;
static const adc_unit_t UNIT = ADC_UNIT_1;

static void ADC_init(void)
{
	adc1_config_width(WIDTH);
	adc1_config_channel_atten(RESISTIVE_STRETCH, ATTEN);
	adc1_config_channel_atten(BATT_VOLT, ATTEN);

	ADC_CHARS = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(UNIT, ATTEN, WIDTH, DEFAULT_VREF, ADC_CHARS);
}

static void GPIO_init(void)
{
	gpio_pad_select_gpio(uC_SENSE);
	gpio_pad_select_gpio(uC_LATCH);
	gpio_pad_select_gpio(GREEN_LED);
	gpio_pad_select_gpio(YELLOW_LED);
	gpio_pad_select_gpio(RED_LED);
	gpio_pad_select_gpio(VIBRATOR_MOTOR);
	//gpio_pad_select_gpio(RESISTIVE_STRETCH);
	//gpio_pad_select_gpio(BATT_VOLT);

	gpio_set_direction(uC_SENSE, GPIO_MODE_INPUT);
	gpio_set_direction(uC_LATCH, GPIO_MODE_OUTPUT);
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(VIBRATOR_MOTOR, GPIO_MODE_OUTPUT);
	//gpio_set_direction(RESISTIVE_STRETCH, GPIO_MODE_INPUT);
	//gpio_set_direction(BATT_VOLT, GPIO_MODE_INPUT);

	gpio_set_level(uC_LATCH, 0);
	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(YELLOW_LED, 0);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(VIBRATOR_MOTOR, 0);
}

static void Resistive_stretch_task(void *pvParameter)
{
	float adc_voltage 	= 0;
	float current 		= 0;
	float voltage 		= 0;
	float resistance 	= 0;
	int adc_reading 	= 0;
	int sens_voltage 	= 0;

	while(1)
	{
		adc_reading = adc1_get_raw((adc1_channel_t)RESISTIVE_STRETCH);
		adc_voltage = esp_adc_cal_raw_to_voltage(adc_reading, ADC_CHARS);
        current = sens_voltage/1000000;
        voltage = (3300-sens_voltage)/1000;
        resistance = voltage/current;
        printf("Stretch sensor resistance: %0.1fOhm\n", resistance);
	}
	vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void Battery_task(void *pvParameter)
{
	uint32_t adc_voltage 	= 0;
	int adc_reading 		= 0;
	int batt_voltage 		= 0;

	while(1)
	{
		adc_reading = adc1_get_raw((adc1_channel_t)BATT_VOLT);
		adc_voltage = esp_adc_cal_raw_to_voltage(adc_reading, ADC_CHARS);
		batt_voltage = 2*adc_voltage;
		if(batt_voltage >= 3800)
		{
			gpio_set_level(YELLOW_LED, 0);
			gpio_set_level(RED_LED, 0);
			gpio_set_level(GREEN_LED, 1);
		}
		else if((batt_voltage > 3400) && (batt_voltage < 3800))
		{
			gpio_set_level(GREEN_LED, 0);
			gpio_set_level(RED_LED, 0);
			gpio_set_level(YELLOW_LED, 1);
		}
		else
		{
			gpio_set_level(GREEN_LED, 0);
			gpio_set_level(YELLOW_LED, 0);
			gpio_set_level(RED_LED, 1);
		}
		vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}


void app_main(void)
{
    GPIO_init();
    ADC_init();

    xTaskCreate(&Resistive_stretch_task, "Resistive stretch sensor", 2048, NULL, 5, NULL);
    xTaskCreate(&Battery_task, "Battery voltage measurement", 2048, NULL, 5, NULL);
}
