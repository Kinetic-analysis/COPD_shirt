/* Timer group-hardware timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <inttypes.h>
#include "driver/pcnt.h"

#define TIMER_DIVIDER         2  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (3.4179) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

volatile int aa = 0;
pcnt_isr_handle_t user_isr_handle = NULL;
/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 *
 *
 */

static void IRAM_ATTR pcnt_isr(void *para)
{
	//timer_pause();
	//aa++;
	uint64_t timer_counter_value;
	timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_counter_value);
	aa = timer_counter_value;
	//printf("%" PRIu64 "\n", timer_counter_value);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
	//timer_start();
	//portYIELD_FROM_ISR();
}

void IRAM_ATTR timer_group0_isr(void *para)
{
//	int16_t blabla;
//	timer_spinlock_take(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, TIMER_0);
//    pcnt_get_counter_value(PCNT_UNIT_0, &blabla);
//    aa = blabla;
//    pcnt_counter_clear(PCNT_UNIT_0);
//
//    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
//    timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, 4000000);
//
//    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
//    timer_spinlock_give(TIMER_GROUP_0);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init()
{
	int bla = TIMER_0;
	printf("Test1\n");
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_DIS,
        .auto_reload = 0,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    printf("Test2\n");
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    printf("Test3\n");
//    /* Configure the alarm value and the interrupt on alarm. */
//    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 4000000);
//    printf("Test4\n");
//    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
//    printf("Test5\n");
//    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, (void *) bla, ESP_INTR_FLAG_IRAM, NULL);
//    printf("Test6\n");
    timer_start(TIMER_GROUP_0, TIMER_0);
    printf("Test7\n");
}


static void pcnt_init()
{
	pcnt_config_t pcnt_config = {
	        .pulse_gpio_num = 2,
	        .ctrl_gpio_num = -1,
	        .lctrl_mode = PCNT_MODE_KEEP,
	        .hctrl_mode = PCNT_MODE_KEEP,
	        .pos_mode = PCNT_COUNT_INC,
			.neg_mode = PCNT_COUNT_DIS,
			.counter_h_lim = 200,
			.counter_l_lim = 0,
			.unit = PCNT_UNIT_0,
			.channel = PCNT_CHANNEL_0
	    };

	pcnt_unit_config(&pcnt_config);

	pcnt_set_filter_value(PCNT_UNIT_0, 1023);
	pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_isr, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);

        /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT_0);
}

void app_main(void)
{
	gpio_pad_select_gpio(33);

	gpio_set_direction(33, GPIO_MODE_OUTPUT);

	gpio_set_level(33, 1); //Zet systeem meteen aan

    example_tg0_timer_init();
    pcnt_init();


    while(1)
    {
    	//printf("%" PRIu64 "\n", aa);
    	printf("%d\n", aa);
    	vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

