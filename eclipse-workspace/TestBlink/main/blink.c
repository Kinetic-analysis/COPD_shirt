#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/timer.h"
#include <inttypes.h>

volatile uint64_t StartValue;
volatile uint64_t PeriodCount;
volatile int64_t starttime;
volatile int knop = 0;

#define BLINK_GPIO CONFIG_BLINK_GPIO

void IRAM_ATTR CapStretch_ISR_Handler(void *arg)
{
	uint64_t counter = 0;
	timer_get_counter_value(TIMER_GROUP_1, TIMER_1, &counter);
	uint64_t TempVal = counter;
	PeriodCount = TempVal - StartValue;
	StartValue = TempVal;
}

void IRAM_ATTR Touchpad_ISR_Handler(void *arg)
{
	knop = 1;
}

static void Cap_NE555_Task(void *pvParameter)
{
	int64_t Value = 0;
	int64_t temp = 0;
	int64_t microseconden = 0;
	while(knop == 1)
	{
		temp = esp_timer_get_time();
		microseconden = temp - starttime;
		Value = PeriodCount;
		printf("%" PRId64 "," "%" PRId64 "\n", microseconden, Value);
		//printf("%" PRId64 "\n", Value);
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void app_main(void)
{
	timer_config_t capSensor;
    capSensor.alarm_en = TIMER_ALARM_DIS;
    capSensor.counter_en = TIMER_START;
    capSensor.intr_type = TIMER_INTR_LEVEL;
    capSensor.counter_dir = TIMER_COUNT_UP;
    capSensor.auto_reload = TIMER_AUTORELOAD_DIS;
    capSensor.divider = 2;

	gpio_pad_select_gpio(GPIO_NUM_32);
	gpio_pad_select_gpio(GPIO_NUM_2);
	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
	gpio_set_intr_type(GPIO_NUM_32, GPIO_INTR_NEGEDGE);
	gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_32, CapStretch_ISR_Handler, NULL);
	gpio_isr_handler_add(GPIO_NUM_2, Touchpad_ISR_Handler, NULL);

	timer_init(TIMER_GROUP_1, TIMER_1, &capSensor);
	while(knop == 0)
	{
		//nothing
	}
	xTaskCreate(&Cap_NE555_Task, "Capacitive Stretch Sensor NE555", 2048, NULL, 5, NULL);
	starttime = esp_timer_get_time();
}
