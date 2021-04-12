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
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "addr_from_stdin.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define DEFAULT_VREF    	3300
#define NO_OF_SAMPLES   	16

#define uC_SENSE			GPIO_NUM_16
#define uC_LATCH			GPIO_NUM_17
#define GREEN_LED			GPIO_NUM_25
#define YELLOW_LED			GPIO_NUM_26
#define RED_LED				GPIO_NUM_27
#define VIBRATOR_MOTOR		GPIO_NUM_33

static esp_adc_cal_characteristics_t *ADC_CHARS;
static const adc_channel_t RESISTIVE_STRETCH = ADC_CHANNEL_6;
static const adc_channel_t BATT_VOLT = ADC_CHANNEL_7;
static const adc_bits_width_t WIDTH = ADC_WIDTH_BIT_12;
static const adc_atten_t ATTEN = ADC_WIDTH_BIT_12;
static const adc_unit_t UNIT = ADC_UNIT_1;

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {
#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(host_ip);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(host_ip, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_in6 dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
#endif
        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(struct sockaddr_in6));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");

        while (1) {
            int err = send(sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }

            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void IRAM_ATTR Power_Latch_ISR_Handler(void *arg)
{
		gpio_set_level(uC_LATCH, 0);
}

static void ADC_init(void)
{
	adc1_config_width(WIDTH);
	adc1_config_channel_atten(RESISTIVE_STRETCH, ATTEN);
	adc1_config_channel_atten(BATT_VOLT, ATTEN);

	ADC_CHARS = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	+esp_adc_cal_characterize(UNIT, ATTEN, WIDTH, DEFAULT_VREF, ADC_CHARS);
}

static void GPIO_init(void)
{
	gpio_pad_select_gpio(uC_SENSE);
	gpio_pad_select_gpio(uC_LATCH);
	gpio_pad_select_gpio(GREEN_LED);
	gpio_pad_select_gpio(YELLOW_LED);
	gpio_pad_select_gpio(RED_LED);

	gpio_set_direction(uC_SENSE, GPIO_MODE_INPUT);
	gpio_set_direction(uC_LATCH, GPIO_MODE_OUTPUT);
	gpio_set_direction(GREEN_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);

	gpio_set_level(GREEN_LED, 0);
	gpio_set_level(YELLOW_LED, 0);
	gpio_set_level(RED_LED, 0);
	gpio_set_level(uC_LATCH, 1);
}

static void Resistive_stretch_task(void *pvParameter)
{
	float adc_voltage 	= 0;
	float current 		= 0;
	float voltage 		= 0;
	float resistance 	= 0;
	int adc_reading 	= 0;

	while(1)
	{
		adc_reading = adc1_get_raw((adc1_channel_t)RESISTIVE_STRETCH);
		adc_voltage = esp_adc_cal_raw_to_voltage(adc_reading, ADC_CHARS);
        current = adc_voltage/1000000;
        voltage = (3300-adc_voltage)/1000;
        resistance = voltage/current;
        printf("adc_reading: %dOhm\n", adc_reading);
        printf("adc_voltage: %0.1fOhm\n", adc_voltage);
        printf("Stretch sensor resistance: %0.1fOhm\n", resistance);
        vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

static void Battery_task(void *pvParameter)
{
	uint32_t adc_voltage 	= 0;
	int adc_reading 		= 0;
	int batt_voltage 		= 0;

	while(1)
	{
		adc_reading = adc1_get_raw((adc1_channel_t)BATT_VOLT);
		adc_voltage = esp_adc_cal_raw_to_voltage(adc_reading, ADC_CHARS);
		batt_voltage = 2*adc_voltage;
		//printf("adc_reading = %d\n", adc_reading);
		//printf("adc_voltage = %d\n", adc_voltage);
		//printf("batt_voltage = %d\n", batt_voltage);
		if(batt_voltage >= 3800)
		{
			gpio_set_level(YELLOW_LED, 0);
			gpio_set_level(RED_LED, 0);
			gpio_set_level(GREEN_LED, 1);
		}
		else if((batt_voltage > 3400) && (batt_voltage < 3800))
		{
			gpio_set_level(GREEN_LED, 0);
			gpio_set_level(RED_LED, 0);
			gpio_set_level(YELLOW_LED, 1);
		}
		else
		{
			gpio_set_level(GREEN_LED, 0);
			gpio_set_level(YELLOW_LED, 0);
			gpio_set_level(RED_LED, 1);
		}
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

static void Motor1_task(void *pvParameter)
{
	while(1)
	{
		//ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
		//printf("0 duty cycle\n");
		//vTaskDelay(5000 / portTICK_PERIOD_MS);
		//ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 64, 0);
		//printf("64 duty cycle\n");
		//vTaskDelay(5000 / portTICK_PERIOD_MS);
		//ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128, 0);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

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
	motor1_channel.duty = 0;
	motor1_channel.hpoint = 0;
	ledc_channel_config(&motor1_channel);
	ledc_fade_func_install(0);
}

void app_main(void)
{
	Motor1_init();
    GPIO_init();
    ADC_init();

	printf("ESP STARTED\n");
	xTaskCreate(&Battery_task, "Battery voltage measurement", 2048, NULL, 5, NULL);
	vTaskDelay(2500 / portTICK_PERIOD_MS);
	gpio_set_intr_type(uC_SENSE, GPIO_INTR_POSEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(uC_SENSE, Power_Latch_ISR_Handler, NULL);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&Resistive_stretch_task, "Resistive stretch sensor", 2048, NULL, 5, NULL);
    xTaskCreate(&tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
    xTaskCreate(&Motor1_task, "Vibrator Motor 1", 2048, NULL, 5, NULL);
}
