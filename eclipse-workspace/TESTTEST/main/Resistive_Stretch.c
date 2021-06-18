/*
 * Resistive_Stretch.c
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */
#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "Resistive_Stretch.h"

float measure_resistance(esp_adc_cal_characteristics_t *adc_stretch_chars, adc_channel_t adc_stretch_channel)
{
	ADC_reading = adc1_get_raw((adc1_channel_t)adc_stretch_channel);
	ADC_voltage = esp_adc_cal_raw_to_voltage(ADC_reading, adc_stretch_chars);
	sensor_current = ADC_voltage/1000000;
	sensor_voltage = (3300-ADC_voltage)/1000;
	sensor_resistance = sensor_voltage/sensor_current;

	return sensor_resistance;
}
