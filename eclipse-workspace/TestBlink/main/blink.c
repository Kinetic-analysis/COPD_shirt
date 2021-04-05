/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
	gpio_pad_select_gpio(GPIO_NUM_25); //Green_LED
	gpio_pad_select_gpio(GPIO_NUM_26); //Yellow_LED
	gpio_pad_select_gpio(GPIO_NUM_27); //Red_LED
    gpio_pad_select_gpio(GPIO_NUM_35); //Battery_VOLTAGE
    gpio_pad_select_gpio(GPIO_NUM_16); //uC_SENSE
    gpio_pad_select_gpio(GPIO_NUM_17); //uC_LATCH
    gpio_pad_select_gpio(GPIO_NUM_33); //Vibrator_MOTOR
    gpio_pad_select_gpio(GPIO_NUM_34); //Resistive_Stretch_OUT
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);	//Green_LED
    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);	//Yellow_LED
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);	//Red_LED
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);	//Battery_VOLTAGE
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_INPUT);	//uC_SENSE
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);	//uC_LATCH
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);	//Vibrator_MOTOR
    gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT);	//Resistive_Stretch_OUT


    gpio_set_level(GPIO_NUM_25, 0);	//Green_LED
    gpio_set_level(GPIO_NUM_26, 0);	//Yellow_LED
    gpio_set_level(GPIO_NUM_27, 0);	//Red_LED
    gpio_set_level(GPIO_NUM_17, 0);	//uC_LATCH
    gpio_set_level(GPIO_NUM_33, 0);	//Vibrator_MOTOR

    while(1) {

//        /* Blink on (output high) */
//        printf("Turning on the LED\n");
//        gpio_set_level(GPIO_NUM_25, 1);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//        /* Blink off (output low) */
//    	printf("Turning off the LED\n");
//        gpio_set_level(GPIO_NUM_25, 0);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//        /* Blink on (output high) */
//        printf("Turning on the LED\n");
//        gpio_set_level(GPIO_NUM_26, 1);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//        /* Blink off (output low) */
//    	printf("Turning off the LED\n");
//        gpio_set_level(GPIO_NUM_26, 0);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//        /* Blink on (output high) */
//        printf("Turning on the LED\n");
//        gpio_set_level(GPIO_NUM_27, 1);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//        /* Blink off (output low) */
//    	printf("Turning off the LED\n");
//        gpio_set_level(GPIO_NUM_27, 0);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
    	gpio_set_level(GPIO_NUM_33, 0);
    	vTaskDelay(2000 / portTICK_PERIOD_MS);
    	gpio_set_level(GPIO_NUM_33, 1);
    	vTaskDelay(500 / portTICK_PERIOD_MS);
    	gpio_set_level(GPIO_NUM_33, 0);
    	vTaskDelay(2000 / portTICK_PERIOD_MS);
    	gpio_set_level(GPIO_NUM_33, 1);
    	vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}