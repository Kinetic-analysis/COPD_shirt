/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_timer.h"
#include <inttypes.h>
#include "esp_pm.h"

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   16          //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel7 = ADC_CHANNEL_7;     //GPIO34 if ADC1
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void app_main(void)
{
	esp_pm_configure(ESP_PM_APB_FREQ_MAX);
	float adc_reading = 0;
	float timingMicros1 = 0;
	float timingMicros2 = 0;
	float b = 0;
	float c = 0;
	float capacity = 0;
	float a = 0;
	float d = 0;
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    adc1_config_width(width);
    adc1_config_channel_atten(channel7, atten);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    gpio_pad_select_gpio(GPIO_NUM_27); //Red_LED
	gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);	//Red_LED
	gpio_set_level(GPIO_NUM_27, 0);	//Red_LED
	//adc_set_clk_div(0);


    //Continuously sample ADC1
    while (1) {
    	timingMicros1 = esp_timer_get_time();
    	gpio_set_level(GPIO_NUM_27, 1);	//Red_LED
    	a = 1;
    	while(a == 1)
    	{
    		adc_reading = adc1_get_raw((adc1_channel_t)channel7);
    		if(esp_adc_cal_raw_to_voltage(adc_reading, adc_chars) >= 2085)
    		{
    			timingMicros2 = esp_timer_get_time();
    			b = timingMicros2-timingMicros1;
    			printf("Tijd = ");
    			printf("%0.2f \t", b);
    			printf("timingMicros1 = %f \t timingMicros2 = %f \t", timingMicros1, timingMicros2);
    			c = b*1000;
    			capacity = c/180000;
    			printf("Capaciteit = ");
    			printf("%0.2f \n", capacity);
    			gpio_set_level(GPIO_NUM_27, 0);	//Red_LED
    			a = 0;
    		}
    	}
    	vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
