#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/timer.h"

volatile uint64_t StartValue;
volatile uint64_t PeriodCount;
volatile int Aan_Uit = 0;

#define uC_SENSE			GPIO_NUM_16
#define uC_LATCH			GPIO_NUM_17
#define GREEN_LED			GPIO_NUM_25
#define YELLOW_LED			GPIO_NUM_26
#define RED_LED				GPIO_NUM_27
#define VIBRATOR_MOTOR		GPIO_NUM_33
#define RESISTIVE_STRETCH	GPIO_NUM_34
#define BATT_VOLT			GPIO_NUM_35

void IRAM_ATTR Power_Latch_ISR_Handler(void *arg)
{
		gpio_set_level(uC_LATCH, 0);
}

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

	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(YELLOW_LED, 0);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(VIBRATOR_MOTOR, 0);
	gpio_set_level(uC_LATCH, 1);

//	gpio_set_intr_type(uC_SENSE, GPIO_INTR_POSEDGE);
//	gpio_install_isr_service(0);
//	gpio_isr_handler_add(uC_SENSE, Power_Latch_ISR_Handler, NULL);
}

void app_main(void)
{
	GPIO_init();
	printf("ESP STARTED\n");
	vTaskDelay(2500 / portTICK_PERIOD_MS);
	gpio_set_intr_type(uC_SENSE, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(uC_SENSE, Power_Latch_ISR_Handler, NULL);

    while(1) {
    	printf("Hello World\n");
    	vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
