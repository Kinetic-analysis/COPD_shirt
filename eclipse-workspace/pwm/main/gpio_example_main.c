/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

static void Motor1_init(void)
{
	ledc_timer_config_t motor1_timer;
	motor1_timer.speed_mode = LEDC_LOW_SPEED_MODE;
	motor1_timer.duty_resolution = LEDC_TIMER_8_BIT;
	motor1_timer.timer_num = LEDC_TIMER_0;
	motor1_timer.freq_hz = 25000;
	motor1_timer.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&motor1_timer);

	ledc_channel_config_t motor1_channel;
	motor1_channel.gpio_num = 33;
	motor1_channel.speed_mode = LEDC_LOW_SPEED_MODE;
	motor1_channel.channel = LEDC_CHANNEL_0;
	motor1_channel.intr_type = LEDC_INTR_DISABLE;
	motor1_channel.timer_sel = LEDC_TIMER_0;
	motor1_channel.duty = 128;
	motor1_channel.hpoint = 0;
	ledc_channel_config(&motor1_channel);

	//ledc_set_duty();
	//ledc_update_duty();

	//ledc_set_pin(33, LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7);
}

static void Motor1_task(void *pvParameter)
{
	uint32_t waarde = 0;
	uint32_t waarde2 = 0;
	while(1)
	{
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
		printf("0 duty cycle\n");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 64, 0);
		printf("64 duty cycle\n");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128, 0);
		printf("128 duty cycle\n");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 192, 0);
		printf("192 duty cycle\n");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 256, 0);
		printf("256 duty cycle\n");
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}


void app_main(void)
{
	Motor1_init();
	ledc_fade_func_install(0);
	xTaskCreate(&Motor1_task, "Vibrator Motor 1", 2048, NULL, 5, NULL);
}

