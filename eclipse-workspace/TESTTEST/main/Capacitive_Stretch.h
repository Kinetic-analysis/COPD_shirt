/*
 * Capacitive_Stretch.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#ifndef MAIN_CAPACITIVE_STRETCH_H_
#define MAIN_CAPACITIVE_STRETCH_H_

#include <stdio.h>
#include "driver/timer.h"
#include "driver/gpio.h"

#define capacitive_stretch_pin 27

void capacitive_stretch_init(void);
uint64_t get_counter(uint64_t *counter);

#endif /* MAIN_CAPACITIVE_STRETCH_H_ */
