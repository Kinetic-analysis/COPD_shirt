/*
 * Vibratormotor.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */

#ifndef MAIN_VIBRATORMOTOR_H_
#define MAIN_VIBRATORMOTOR_H_

#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

#define Vibrator_motor1_pin		GPIO_NUM_22
#define Vibrator_motor2_pin		GPIO_NUM_26

void vibrator_motor_init(void);
void vibratormotor_on(uint32_t dutycycle);

#endif /* MAIN_VIBRATORMOTOR_H_ */
