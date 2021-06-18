/*
 * Capacitive_Stretch.c
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#include <stdio.h>
#include "driver/timer.h"
#include "driver/gpio.h"
#include "Capacitive_Stretch.h"

void capacitive_stretch_init(void)
{
	timer_config_t capSensor;
    capSensor.alarm_en = TIMER_ALARM_DIS;
    capSensor.counter_en = TIMER_START;
    capSensor.intr_type = TIMER_INTR_LEVEL;
    capSensor.counter_dir = TIMER_COUNT_UP;
    capSensor.auto_reload = TIMER_AUTORELOAD_DIS;
    capSensor.divider = 2;

    timer_init(TIMER_GROUP_1, TIMER_1, &capSensor);

	gpio_pad_select_gpio(capacitive_stretch_pin);
	gpio_set_direction(capacitive_stretch_pin, GPIO_MODE_INPUT);
}

uint64_t get_counter(uint64_t *counter)
{
	timer_get_counter_value(TIMER_GROUP_1, TIMER_1, &counter);

	return counter;
}
