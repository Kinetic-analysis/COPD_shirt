/*
 * Touchpad.c
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */
#include <stdio.h>
#include "driver/touch_pad.h"
#include "Touchpad.h"

void touchpad_init(void)
{
    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);
    touch_pad_set_cnt_mode(TOUCH_PAD_NUM2, TOUCH_PAD_SLOPE_1, TOUCH_PAD_TIE_OPT_LOW);
    touch_pad_set_meas_time(0x0001, 0xffff);
    touch_pad_config(TOUCH_PAD_NUM2, TOUCH_THRESH_NO_USE);
}

uint16_t read_touchpad(void)
{
	touch_pad_read(TOUCH_PAD_NUM2, &touchpad_value);
	return touchpad_value;
}
