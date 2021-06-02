/*
 * Speaker.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#ifndef MAIN_SPEAKER_H_
#define MAIN_SPEAKER_H_

#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

void speaker_init(void);
void speaker_on(uint32_t frequency, int on_off);

#endif /* MAIN_SPEAKER_H_ */
