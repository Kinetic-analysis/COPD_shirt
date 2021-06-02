/*
 * Vibratormotor.c
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "Vibratormotor.h"

void vibrator_motor_init(void)
{
	ledc_timer_config_t Vibrator_motor_timer1;
	Vibrator_motor_timer1.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_timer1.duty_resolution = LEDC_TIMER_8_BIT;
	Vibrator_motor_timer1.timer_num = LEDC_TIMER_0;
	Vibrator_motor_timer1.freq_hz = 25000;
	Vibrator_motor_timer1.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&Vibrator_motor_timer1);

	ledc_channel_config_t Vibrator_motor_channel1;
	Vibrator_motor_channel1.gpio_num = 22;
	Vibrator_motor_channel1.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_channel1.channel = LEDC_CHANNEL_0;
	Vibrator_motor_channel1.intr_type = LEDC_INTR_DISABLE;
	Vibrator_motor_channel1.timer_sel = LEDC_TIMER_0;
	Vibrator_motor_channel1.duty = 0;
	Vibrator_motor_channel1.hpoint = 0;
	ledc_channel_config(&Vibrator_motor_channel1);

	ledc_timer_config_t Vibrator_motor_timer2;
	Vibrator_motor_timer2.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_timer2.duty_resolution = LEDC_TIMER_8_BIT;
	Vibrator_motor_timer2.timer_num = LEDC_TIMER_0;
	Vibrator_motor_timer2.freq_hz = 25000;
	Vibrator_motor_timer2.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&Vibrator_motor_timer2);

	ledc_channel_config_t Vibrator_motor_channel2;
	Vibrator_motor_channel2.gpio_num = 26;
	Vibrator_motor_channel2.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_channel2.channel = LEDC_CHANNEL_0;
	Vibrator_motor_channel2.intr_type = LEDC_INTR_DISABLE;
	Vibrator_motor_channel2.timer_sel = LEDC_TIMER_0;
	Vibrator_motor_channel2.duty = 0;
	Vibrator_motor_channel2.hpoint = 0;
	ledc_channel_config(&Vibrator_motor_channel2);

	ledc_fade_func_install(0);
}

void vibratormotor_on(uint32_t dutycycle)
{
	ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dutycycle, 0);
}
