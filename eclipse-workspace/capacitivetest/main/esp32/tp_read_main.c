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

#define Power_sense_pin			GPIO_NUM_34
#define Power_on_off_pin		GPIO_NUM_33
#define Green_led_pin			GPIO_NUM_17
#define Yellow_led_pin			GPIO_NUM_16
#define Red_led_pin				GPIO_NUM_21
#define Vibrator_motor1_pin		GPIO_NUM_22
#define Vibrator_motor2_pin		GPIO_NUM_26
#define Touch_sensor			GPIO_NUM_2
#define Battery					GPIO_NUM_35

volatile uint64_t StartValue = 0;
volatile uint64_t PeriodCount = 0;
//volatile uint64_t counter;

void IRAM_ATTR Touchpad_ISR_Handler(void *arg)
{
	gpio_set_level(Power_on_off_pin, 0);
}

void IRAM_ATTR CapStretch_ISR_Handler(void *arg)
{
	uint64_t counter = 0;
	timer_get_counter_value(TIMER_GROUP_1, TIMER_1, &counter);
	uint64_t TempVal = counter;
	PeriodCount = TempVal - StartValue;
	StartValue = TempVal;
	//printf("KUTZOOI\n");
}

static void Cap_NE555_Task(void *pvParameter)
{
	int64_t Value = 0;
	int64_t temp = 0;
	int64_t microseconden = 0;
	while(1)
	{
		temp = esp_timer_get_time();
		microseconden = temp - starttime;
		Value = PeriodCount;
		printf("%" PRId64 "," "%" PRId64 "\n", microseconden, Value);
		//printf("%" PRId64 "\n", Value);
		//printf("Hallo!\n");
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

static void GPIO_init(void)
{
	gpio_pad_select_gpio(Power_sense_pin);
	gpio_pad_select_gpio(Power_on_off_pin);
	gpio_pad_select_gpio(Green_led_pin);
	gpio_pad_select_gpio(Yellow_led_pin);
	gpio_pad_select_gpio(Red_led_pin);
	gpio_pad_select_gpio(Battery);

	gpio_set_direction(Power_sense_pin, GPIO_MODE_INPUT);
	gpio_set_direction(Power_on_off_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Green_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Yellow_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Red_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Battery, GPIO_MODE_INPUT);

	gpio_set_level(Green_led_pin, 1);
	gpio_set_level(Yellow_led_pin, 0);
	gpio_set_level(Red_led_pin, 0);
	gpio_set_level(Power_on_off_pin, 1); //Zet systeem meteen aan
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

    	touch_pad_read(TOUCH_PAD_NUM7, &touch_filter_value);

        printf("%" PRId64 ", %4d\n", microseconden, touch_filter_value);
    	//printf("%4d\n", touch_filter_value);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void tp_example_touch_pad_init(void)
{
    touch_pad_config(TOUCH_PAD_NUM7, TOUCH_THRESH_NO_USE);
}

void app_main(void)
{
	GPIO_init();
    touch_pad_init();

    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);
    touch_pad_set_cnt_mode(TOUCH_PAD_NUM7, TOUCH_PAD_SLOPE_1, TOUCH_PAD_TIE_OPT_LOW);
    touch_pad_set_meas_time(0x0001, 0xffff);

    tp_example_touch_pad_init();

	timer_config_t capSensor;
    capSensor.alarm_en = TIMER_ALARM_DIS;
    capSensor.counter_en = TIMER_START;
    capSensor.intr_type = TIMER_INTR_LEVEL;
    capSensor.counter_dir = TIMER_COUNT_UP;
    capSensor.auto_reload = TIMER_AUTORELOAD_DIS;
    capSensor.divider = 2;

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    timer_init(TIMER_GROUP_1, TIMER_1, &capSensor);
	gpio_pad_select_gpio(GPIO_NUM_32);
	//gpio_pad_select_gpio(Power_sense_pin);
	gpio_set_direction(GPIO_NUM_32, GPIO_MODE_INPUT);
	//gpio_set_direction(Power_sense_pin, GPIO_MODE_INPUT);
	gpio_set_intr_type(GPIO_NUM_32, GPIO_INTR_NEGEDGE);
	//gpio_set_intr_type(Power_sense_pin, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_NUM_32, CapStretch_ISR_Handler, NULL);
	//gpio_isr_handler_add(Power_sense_pin, Touchpad_ISR_Handler, NULL);




    // Start task to read values sensed by pads
    //xTaskCreate(&tp_example_read_task, "touch_pad_read_task", 2048, NULL, 5, NULL);
	xTaskCreate(&Cap_NE555_Task, "Capacitive Stretch Sensor NE555", 2048, NULL, 5, NULL);
    starttime = esp_timer_get_time();
    //xTaskCreate(&tp_example_led_task, "touch_pad_led_task", 2048, NULL, 4, NULL);
}
