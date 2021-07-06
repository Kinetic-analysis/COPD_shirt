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
	new_voltage = (1.0074074*ADC_voltage)-42.2667;
//	new_voltage = ADC_voltage;
	sensor_current = new_voltage/1000000;
	sensor_voltage = (3300-new_voltage)/1000;
	sensor_resistance = sensor_voltage/sensor_current;
//	printf("ADC Reading: %d\t ADC Voltage: %f\t Resistance: %f\n", ADC_reading, new_voltage, sensor_resistance);

	return sensor_resistance;
}
