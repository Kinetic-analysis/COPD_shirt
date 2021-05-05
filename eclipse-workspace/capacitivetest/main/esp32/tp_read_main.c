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
#include "driver/timer.h"
#include <inttypes.h>

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (1)
volatile int64_t starttime;
volatile int knop = 0;

void IRAM_ATTR Touchpad_ISR_Handler(void *arg)
{
	knop = 1;
}

static void tp_example_led_task(void *pvParameter)
{
	while(1)
	{
		gpio_set_level(GPIO_NUM_4, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_4, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

/*
  Read values sensed at all available touch pads.
 Print out values in a loop on a serial monitor.
 */
static void tp_example_read_task(void *pvParameter)
{
	uint16_t counter = 0;
    uint16_t touch_value;
    uint16_t touch_filter_value;
    while (1) {
    	int64_t temp = 0;
    	int64_t microseconden = 0;
    	temp = esp_timer_get_time();
    	microseconden = temp - starttime;

    	touch_pad_read(TOUCH_PAD_NUM3, &touch_filter_value);

        printf("%" PRId64 ", %4d\n", microseconden, touch_filter_value);
    	//printf("%4d\n", touch_filter_value);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void tp_example_touch_pad_init(void)
{
    touch_pad_config(TOUCH_PAD_NUM3, TOUCH_THRESH_NO_USE);
}

void app_main(void)
{

	gpio_pad_select_gpio(GPIO_NUM_4); //Green_LED
	gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_4, 0);
    // Initialize touch pad peripheral.
    // The default fsm mode is software trigger mode.
    touch_pad_init();
    // Set reference voltage for charging/discharging
    // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
    // The low reference voltage will be 0.5
    // The larger the range, the larger the pulse count value.
    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);
    touch_pad_set_cnt_mode(TOUCH_PAD_NUM3, TOUCH_PAD_SLOPE_1, TOUCH_PAD_TIE_OPT_LOW);
    touch_pad_set_meas_time(0x0001, 0xffff);

    tp_example_touch_pad_init();

	timer_config_t capSensor;
    capSensor.alarm_en = TIMER_ALARM_DIS;
    capSensor.counter_en = TIMER_START;
    capSensor.intr_type = TIMER_INTR_LEVEL;
    capSensor.counter_dir = TIMER_COUNT_UP;
    capSensor.auto_reload = TIMER_AUTORELOAD_DIS;
    capSensor.divider = 2;

	//gpio_pad_select_gpio(GPIO_NUM_32);
	gpio_pad_select_gpio(GPIO_NUM_2);
	//gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
	//gpio_set_intr_type(GPIO_NUM_32, GPIO_INTR_NEGEDGE);
	gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	//gpio_isr_handler_add(GPIO_NUM_32, CapStretch_ISR_Handler, NULL);
	gpio_isr_handler_add(GPIO_NUM_2, Touchpad_ISR_Handler, NULL);

	timer_init(TIMER_GROUP_1, TIMER_1, &capSensor);
	while(knop == 0)
	{
		//nothing
	}

    // Start task to read values sensed by pads
    xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
    starttime = esp_timer_get_time();
    //xTaskCreate(&tp_example_led_task, "touch_pad_led_task", 2048, NULL, 4, NULL);
}
