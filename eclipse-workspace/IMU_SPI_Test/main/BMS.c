/*
 * BMS.c
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "BMS.h"

void gpio_bms_init(void)
{
	gpio_pad_select_gpio(power_sense_pin);
	gpio_pad_select_gpio(power_on_off_pin);
	gpio_pad_select_gpio(green_led_pin);
	gpio_pad_select_gpio(yellow_led_pin);
	gpio_pad_select_gpio(red_led_pin);

	gpio_set_direction(power_sense_pin, GPIO_MODE_INPUT);
	gpio_set_direction(power_on_off_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(green_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(yellow_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(red_led_pin, GPIO_MODE_OUTPUT);

	gpio_set_level(green_led_pin, 0);
	gpio_set_level(yellow_led_pin, 0);
	gpio_set_level(red_led_pin, 0);
	gpio_set_level(power_on_off_pin, 1); //Zet systeem meteen aan
}

void leds_on_off(int green_led_status, int yellow_led_status, int red_led_status)
{
	gpio_set_level(green_led_pin, green_led_status);
	gpio_set_level(yellow_led_pin, yellow_led_status);
	gpio_set_level(red_led_pin, red_led_status);
}

int measure_battery_voltage(const esp_adc_cal_characteristics_t *adc_bms_chars, adc_channel_t adc_bms_channel)
{
	adc_reading = adc1_get_raw((adc1_channel_t)adc_bms_channel);
	adc_voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_bms_chars);
	battery_voltage = 2*adc_voltage;
	return battery_voltage;
}

void control_leds(int battery_voltage)
{
	if(battery_voltage >= 3800)
	{
		leds_on_off(1,0,0);
	}
	if((battery_voltage >= 3400) && (battery_voltage < 3800))
	{
		leds_on_off(0,1,0);
	}
	if((battery_voltage >= 3000) && (battery_voltage < 3400))
	{
		leds_on_off(0,0,1);
	}
	if((battery_voltage < 3000))
	{
		gpio_set_level(power_on_off_pin, 0);
	}
}
