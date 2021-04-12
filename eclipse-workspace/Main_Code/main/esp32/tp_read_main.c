/* Touch Pad Read Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define uC_SENSE			GPIO_NUM_16
#define uC_LATCH			GPIO_NUM_17
#define GREEN_LED			GPIO_NUM_25
#define YELLOW_LED			GPIO_NUM_26
#define RED_LED				GPIO_NUM_27
#define VIBRATOR_MOTOR		GPIO_NUM_33
#define RESISTIVE_STRETCH	GPIO_NUM_34
#define BATT_VOLT			GPIO_NUM_35

static void GPIO_init(void)
{
	gpio_pad_select_gpio(uC_SENSE);
	gpio_pad_select_gpio(uC_LATCH);
	gpio_pad_select_gpio(GREEN_LED);
	gpio_pad_select_gpio(YELLOW_LED);
	gpio_pad_select_gpio(RED_LED);
	gpio_pad_select_gpio(VIBRATOR_MOTOR);
	gpio_pad_select_gpio(RESISTIVE_STRETCH);
	gpio_pad_select_gpio(BATT_VOLT);

	gpio_set_direction(uC_SENSE, GPIO_MODE_INPUT);
	gpio_set_direction(uC_LATCH, GPIO_MODE_OUTPUT);
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(VIBRATOR_MOTOR, GPIO_MODE_OUTPUT);
	gpio_set_direction(RESISTIVE_STRETCH, GPIO_MODE_INPUT);
	gpio_set_direction(BATT_VOLT, GPIO_MODE_INPUT);

	gpio_set_level(uC_LATCH, 0);
	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(YELLOW_LED, 0);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(VIBRATOR_MOTOR, 0);
}

static void LED_task(void *pvParameter)
{
	gpio_set_level(YELLOW_LED, 0);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(GREEN_LED, 1);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(YELLOW_LED, 1);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(YELLOW_LED, 0);
	gpio_set_level(RED_LED, 1);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}


void app_main(void)
{
    GPIO_init();

    xTaskCreate(&LED_task, "RGY Led control", 2048, NULL, 5, NULL);
}
