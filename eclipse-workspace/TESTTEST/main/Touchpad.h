/*
 * Touchpad.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#ifndef MAIN_TOUCHPAD_H_
#define MAIN_TOUCHPAD_H_

#include <stdio.h>
#include "driver/touch_pad.h"

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (1)

uint16_t touchpad_value;

void touchpad_init(void);
uint16_t read_touchpad(void);

#endif /* MAIN_TOUCHPAD_H_ */
