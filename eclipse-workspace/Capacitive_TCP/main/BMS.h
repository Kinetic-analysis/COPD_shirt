/*
 * BMS.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#ifndef MAIN_BMS_H_
#define MAIN_BMS_H_

#include <stdio.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define power_sense_pin			GPIO_NUM_34
#define power_on_off_pin		GPIO_NUM_33
#define green_led_pin			GPIO_NUM_17
#define yellow_led_pin			GPIO_NUM_16
#define red_led_pin				GPIO_NUM_21

uint32_t adc_voltage;
int adc_reading;
int battery_voltage;

void gpio_bms_init(void);
void leds_on_off(int green_led_status, int yellow_led_status, int red_led_status);
int measure_battery_voltage(const esp_adc_cal_characteristics_t *adc_bms_chars, adc_channel_t adc_bms_channel);
void control_leds(int battery_voltage);
#endif /* MAIN_BMS_H_ */
