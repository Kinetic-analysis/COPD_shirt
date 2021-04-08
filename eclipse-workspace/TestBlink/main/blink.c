#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/timer.h"

volatile uint64_t StartValue;
volatile uint64_t PeriodCount;

#define BLINK_GPIO CONFIG_BLINK_GPIO

void IRAM_ATTR CapStretch_ISR_Handler(void *arg)
{
	uint64_t counter = 0;
	timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &counter);
	uint64_t TempVal = counter;
	PeriodCount = TempVal - StartValue;
	StartValue = TempVal;
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

	gpio_pad_select_gpio(GPIO_NUM_23);
	gpio_set_direction(GPIO_NUM_23, GPIO_MODE_INPUT);
	gpio_set_intr_type(GPIO_NUM_23, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_23, CapStretch_ISR_Handler, NULL);

	timer_init(TIMER_GROUP_0, TIMER_0, &capSensor);

	float Value = 0;

    while(1) {
    	Value = PeriodCount;
    	printf("%0.1f\n", Value);
    	vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
