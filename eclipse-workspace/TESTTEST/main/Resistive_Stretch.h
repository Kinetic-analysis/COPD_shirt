/*
 * Resistive_Stretch.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#ifndef MAIN_RESISTIVE_STRETCH_H_
#define MAIN_RESISTIVE_STRETCH_H_

#include <stdio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

float ADC_voltage;
float new_voltage;
float sensor_current;
float sensor_voltage;
float sensor_resistance;
int ADC_reading;

float measure_resistance(esp_adc_cal_characteristics_t *adc_stretch_chars, adc_channel_t adc_stretch_channel);

#endif /* MAIN_RESISTIVE_STRETCH_H_ */
