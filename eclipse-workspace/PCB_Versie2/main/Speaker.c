/*
 * Speaker.c
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */
#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "Speaker.h"

void speaker_init(void)
{
	ledc_timer_config_t Speaker_timer;
	Speaker_timer.speed_mode = LEDC_LOW_SPEED_MODE;
	Speaker_timer.duty_resolution = LEDC_TIMER_8_BIT;
	Speaker_timer.timer_num = LEDC_TIMER_1;
	Speaker_timer.freq_hz = 1000;
	Speaker_timer.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&Speaker_timer);

	ledc_channel_config_t Speaker_channel;
	Speaker_channel.gpio_num = 25;
	Speaker_channel.speed_mode = LEDC_LOW_SPEED_MODE;
	Speaker_channel.channel = LEDC_TIMER_1;
	Speaker_channel.intr_type = LEDC_INTR_DISABLE;
	Speaker_channel.timer_sel = LEDC_TIMER_1;
	Speaker_channel.duty = 128;
	Speaker_channel.hpoint = 0;
	ledc_channel_config(&Speaker_channel);
}

void speaker_on(uint32_t frequency, int on_off)
{
	if(on_off == 1)
	{
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 128, 0);
	}else
	{
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0, 0);
	}
	ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_1, frequency);
}
