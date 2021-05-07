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
#include "driver/timer.h"
#include <inttypes.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdbool.h>
#include "driver/touch_pad.h"
#include <inttypes.h>

#define TOUCH_PAD_NO_CHANGE   (-1)
#define TOUCH_THRESH_NO_USE   (0)
#define TOUCH_FILTER_MODE_EN  (1)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (1)

#define DEFAULT_VREF    	3300
#define NO_OF_SAMPLES   	16

//GPIO pins
#define Power_sense_pin			GPIO_NUM_34
#define Power_on_off_pin		GPIO_NUM_33
#define Green_led_pin			GPIO_NUM_17
#define Yellow_led_pin			GPIO_NUM_16
#define Red_led_pin				GPIO_NUM_21
#define Vibrator_motor1_pin		GPIO_NUM_22
#define Vibrator_motor2_pin		GPIO_NUM_26
#define Touch_sensor			GPIO_NUM_2



#define IMU_HOST    VSPI_HOST
#define DMA_CHAN    2

//SPI pins
#define Spi_miso_pin 	19
#define Spi_mosi_pin 	23
#define Spi_clk_pin  	18
#define Spi_imu_cs   	5
#define Spi_sd_cs		4

#define PARALLEL_LINES 16

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

static esp_adc_cal_characteristics_t *ADC_characteristics;
static const adc_channel_t Resistive_stretch1 = ADC_CHANNEL_0;
static const adc_channel_t Resistive_stretch2 = ADC_CHANNEL_3;
static const adc_channel_t Battery_voltage = ADC_CHANNEL_7;
static const adc_bits_width_t ADC_resolution = ADC_WIDTH_BIT_12;
static const adc_atten_t ADC_attenuation = ADC_ATTEN_DB_11;
static const adc_unit_t ADC_unit = ADC_UNIT_1;

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

volatile int64_t start_measure_time;
volatile int start_measure = 0;

void IRAM_ATTR Power_on_off_ISR_Handler(void *arg)
{
		gpio_set_level(Power_on_off_pin, 0);
}


void IRAM_ATTR Touchpad_ISR_Handler(void *arg)
{
	start_measure = 1;
}

bool IMU_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data)
{
	spi_transaction_t t;
	uint8_t buf[2];

	gpio_set_level(Spi_imu_cs, 0);

	reg &= 0x7f;
	buf[0] = reg;
	buf[1] = data;
	memset(&t, 0, sizeof(t));
	t.length = 8*2;
	t.tx_buffer = buf;

	spi_device_transmit(spi, &t);
	gpio_set_level(Spi_imu_cs, 1);
	return true;
}

bool IMU_read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *data)
{
	spi_transaction_t t;
	gpio_set_level(Spi_imu_cs, 0);

	reg = ((reg & 0x7F) | 0x80);
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.tx_buffer = &reg;
	spi_device_transmit(spi, &t);

	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.flags = SPI_TRANS_USE_RXDATA;
	spi_device_transmit(spi, &t);

	gpio_set_level(Spi_imu_cs, 1);
	*data = t.rx_data[0];

	return true;
}

void IMU_read_ID(spi_device_handle_t spi)
{
	//Leest de ID van de IMU sensor
	//De ID van de IMU is 0xAE
	uint8_t reg = 0x00;
	uint8_t id;

	uint8_t write;
	uint8_t read;

	//write REG_BANK_SEL
	reg = 0x7F;
	write = 0x00;
	IMU_write_reg(spi, reg, write);
	IMU_read_reg(spi, reg, &read);
	printf("REG_BANK_SEL: 0x%02X\n", read);

	reg = 0x00;
	IMU_read_reg(spi, reg, &id);
	printf("IMU ID: 0x%02X\n", id);
}

spi_device_handle_t SPI_init(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=Spi_miso_pin,
        .mosi_io_num=Spi_mosi_pin,
        .sclk_io_num=Spi_clk_pin,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=PARALLEL_LINES*320*2+8
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1000000,           		//Clock out at 1 MHz
        .mode=0,                                //SPI mode 3
        .spics_io_num=-1,               		//CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(IMU_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(IMU_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(Spi_imu_cs, GPIO_MODE_OUTPUT);
    gpio_set_direction(Spi_sd_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(Spi_imu_cs, 1);				//CS = Hoog
    gpio_set_level(Spi_sd_cs, 1);				//CS = Hoog
    return spi;
}

void IMU_init_Magneto(spi_device_handle_t spi)
{
	//Initialiseert de magneto om te communiceren met de ICM-20498 via I2C
	//De magnetometer data wordt in de externe slave registers geplaatst
	uint8_t read[12];
	uint8_t reg[12] = {0x7F, 0x03, 0x7F, 0x01, 0x02, 0x03, 0x04, 0x06, 0x05, 0x03, 0x04, 0x7F};
	uint8_t write[12] = {0x00, 0x20, 0x30, 0x17, 0x01, 0x0C, 0x31, 0x08, 0x8A, 0x8C, 0x11, 0x00};

	for(int i = 0; i < 12; i++)
	{
		IMU_write_reg(spi, reg[i], write[i]);
		IMU_read_reg(spi, reg[i], &read[i]);
	}
}

void IMU_read_data(spi_device_handle_t spi)
{
	uint8_t reg[28] = {0x39, 0x3A, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
					   0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48};
	uint8_t data_HL[28];
	int16_t data_OUT[14];
	float data_Final[14];

	for(int i = 0; i < 28; i++)
	{
		IMU_read_reg(spi, reg[i], &data_HL[i]);	//Leest alle sensor registers uit
	}

	data_OUT[0] = ((data_HL[0] << 8) | (data_HL[1] & 0xFF));	//Combineert high en low byte van temperatuur sensor met two's complement
	data_OUT[1] = ((data_HL[2] << 8) | (data_HL[3] & 0xFF));	//Combineert high en low byte van accelerometer met two's complement
	data_OUT[2] = ((data_HL[4] << 8) | (data_HL[5] & 0xFF));	//Combineert high en low byte van accelerometer met two's complement
	data_OUT[3] = ((data_HL[6] << 8) | (data_HL[7] & 0xFF));	//Combineert high en low byte van accelerometer met two's complement
	data_OUT[4] = ((data_HL[8] << 8) | (data_HL[9] & 0xFF));	//Combineert high en low byte van gyroscope met two's complement
	data_OUT[5] = ((data_HL[10] << 8) | (data_HL[11] & 0xFF));	//Combineert high en low byte van gyroscope met two's complement
	data_OUT[6] = ((data_HL[12] << 8) | (data_HL[13] & 0xFF));	//Combineert high en low byte van gyroscope met two's complement

	data_OUT[7] = ((data_HL[15] << 8) | (data_HL[14] & 0xFF));	//Combineert high en low byte van Magnetometer X-as met Little Endian
	data_OUT[8] = ((data_HL[17] << 8) | (data_HL[16] & 0xFF));	//Combineert high en low byte van Magnetometer Y-as met Little Endian
	data_OUT[9] = ((data_HL[19] << 8) | (data_HL[18] & 0xFF));	//Combineert high en low byte van Magnetometer Z-as met Little Endian

	data_Final[0] = ((data_OUT[0]-20)/333.87)+21;	//Berekend de temperatuur in graden celcius
	data_Final[1] = data_OUT[1]/16.384;				//Berekend de accelerometer waarde van de X-axis in mg
	data_Final[2] = data_OUT[2]/16.384;				//Berekend de accelerometer waarde van de Y-axis in mg
	data_Final[3] = data_OUT[3]/16.384;				//Berekend de accelerometer waarde van de Z-axis in mg
	data_Final[4] = data_OUT[4]/131;				//Berekend de gyroscope waarde van de X-axis in dps
	data_Final[5] = data_OUT[5]/131;				//Berekend de gyroscope waarde van de Y-axis in dps
	data_Final[6] = data_OUT[6]/131;				//Berekend de gyroscope waarde van de Z-axis in dps
	data_Final[7] = data_OUT[7]*0.15;				//Berekend de magnetometer waarde van de X-axis in uT
	data_Final[8] = data_OUT[8]*0.15;				//Berekend de magnetometer waarde van de Y-axis in uT
	data_Final[9] = data_OUT[9]*0.15;				//Berekend de magnetometer waarde van de Z-axis in uT

	printf("Temp (C) = [%0.2f]\t "
			"Accel XYZ (mg) = [%0.2f, %0.2f, %0.2f]\t "
			"Gyro XYZ (dps) = [%0.2f, %0.2f, %0.2f]\t "
			"Magn XYZ (uT) = [%0.2f, %0.2f, %0.2f]\n"
			, data_Final[0],
			data_Final[1], data_Final[2], data_Final[3],
			data_Final[4], data_Final[5], data_Final[6],
			data_Final[7], data_Final[8], data_Final[9]);
}

static void GPIO_init(void)
{
	gpio_pad_select_gpio(Power_sense_pin);
	gpio_pad_select_gpio(Power_on_off_pin);
	gpio_pad_select_gpio(Green_led_pin);
	gpio_pad_select_gpio(Yellow_led_pin);
	gpio_pad_select_gpio(Red_led_pin);

	gpio_set_direction(Power_sense_pin, GPIO_MODE_INPUT);
	gpio_set_direction(Power_on_off_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Green_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Yellow_led_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(Red_led_pin, GPIO_MODE_OUTPUT);

	gpio_set_level(Green_led_pin, 0);
	gpio_set_level(Yellow_led_pin, 0);
	gpio_set_level(Red_led_pin, 0);
	gpio_set_level(Power_on_off_pin, 1); //Zet systeem meteen aan
}

static void Interrupt_init(void)
{
	gpio_set_intr_type(Power_sense_pin, GPIO_INTR_POSEDGE);
	gpio_pad_select_gpio(GPIO_NUM_2);
	gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
	gpio_set_intr_type(GPIO_NUM_2, GPIO_INTR_NEGEDGE);
	gpio_install_isr_service(0);
	gpio_isr_handler_add(Power_sense_pin, Power_on_off_ISR_Handler, NULL);
	gpio_isr_handler_add(GPIO_NUM_2, Touchpad_ISR_Handler, NULL);
}

static void ADC_init(void)
{
	adc1_config_width(ADC_resolution);
	adc1_config_channel_atten(Resistive_stretch1, ADC_attenuation);
	adc1_config_channel_atten(Resistive_stretch2, ADC_attenuation);
	adc1_config_channel_atten(Battery_voltage, ADC_attenuation);

	ADC_characteristics = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_characterize(ADC_unit, ADC_attenuation, ADC_resolution, DEFAULT_VREF, ADC_characteristics);
}

static void Vibrator_motor_init(void)
{
	//De trilmotor GPIO wordt geinitialiseerd met een frequentie van 25kHz en een resolutie van 8 bits
	ledc_timer_config_t Vibrator_motor_timer1;
	Vibrator_motor_timer1.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_timer1.duty_resolution = LEDC_TIMER_8_BIT;
	Vibrator_motor_timer1.timer_num = LEDC_TIMER_0;
	Vibrator_motor_timer1.freq_hz = 25000;
	Vibrator_motor_timer1.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&Vibrator_motor_timer1);

	ledc_channel_config_t Vibrator_motor_channel1;
	Vibrator_motor_channel1.gpio_num = 22;
	Vibrator_motor_channel1.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_channel1.channel = LEDC_CHANNEL_0;
	Vibrator_motor_channel1.intr_type = LEDC_INTR_DISABLE;
	Vibrator_motor_channel1.timer_sel = LEDC_TIMER_0;
	Vibrator_motor_channel1.duty = 0;
	Vibrator_motor_channel1.hpoint = 0;
	ledc_channel_config(&Vibrator_motor_channel1);

	ledc_timer_config_t Vibrator_motor_timer2;
	Vibrator_motor_timer2.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_timer2.duty_resolution = LEDC_TIMER_8_BIT;
	Vibrator_motor_timer2.timer_num = LEDC_TIMER_0;
	Vibrator_motor_timer2.freq_hz = 25000;
	Vibrator_motor_timer2.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&Vibrator_motor_timer2);

	ledc_channel_config_t Vibrator_motor_channel2;
	Vibrator_motor_channel2.gpio_num = 26;
	Vibrator_motor_channel2.speed_mode = LEDC_LOW_SPEED_MODE;
	Vibrator_motor_channel2.channel = LEDC_CHANNEL_0;
	Vibrator_motor_channel2.intr_type = LEDC_INTR_DISABLE;
	Vibrator_motor_channel2.timer_sel = LEDC_TIMER_0;
	Vibrator_motor_channel2.duty = 0;
	Vibrator_motor_channel2.hpoint = 0;
	ledc_channel_config(&Vibrator_motor_channel2);

	ledc_fade_func_install(0);
}

// ************************ //
// TCP Client initialisatie //
// ************************ //
static void tcp_client_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
}

// ************** //
// Trilmotor taak //
// ************** //
static void Motor1_task(void *pvParameter)
{
	while(1)
	{
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 0);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128, 0);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

// ************************ //
// TCP Client taak //
// ************************ //
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
	while(1){
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}



// ***************************** //
// Resistive Stretch Sensor taak //
// ***************************** //
static void Resistive_stretch1_task(void *pvParameter)
{
	float ADC_voltage 	= 0;
	float Sensor_current 		= 0;
	float Sensor_voltage 		= 0;
	float Sensor_resistance 	= 0;
	int ADC_reading 	= 0;
	int64_t temp = 0;
	int64_t microseconden = 0;

	while(start_measure == 1)
	{
		ADC_reading = adc1_get_raw((adc1_channel_t)Resistive_stretch1);
		ADC_voltage = esp_adc_cal_raw_to_voltage(ADC_reading, ADC_characteristics);
		Sensor_current = ADC_voltage/1000000;
        Sensor_voltage = (3300-ADC_voltage)/1000;
        Sensor_resistance = Sensor_voltage/Sensor_current;
        temp = esp_timer_get_time();
        microseconden = temp - start_measure_time;
        printf("%" PRId64 ", %0.1fOhm\n", microseconden, Sensor_resistance);
        vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static void Resistive_stretch2_task(void *pvParameter)
{
	float ADC_voltage 	= 0;
	float Sensor_current 		= 0;
	float Sensor_voltage 		= 0;
	float Sensor_resistance 	= 0;
	int ADC_reading 	= 0;
	int64_t temp = 0;
	int64_t microseconden = 0;

	while(start_measure == 1)
	{
		ADC_reading = adc1_get_raw((adc1_channel_t)Resistive_stretch2);
		ADC_voltage = esp_adc_cal_raw_to_voltage(ADC_reading, ADC_characteristics);
        Sensor_current = ADC_voltage/1000000;
        Sensor_voltage = (3300-ADC_voltage)/1000;
        Sensor_resistance = Sensor_voltage/Sensor_current;
        temp = esp_timer_get_time();
        microseconden = temp - start_measure_time;
        printf("%" PRId64 ", %0.1fOhm\n", microseconden, Sensor_resistance);
        vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static void Control_led(int Green_led_status, int Yellow_led_status, int Red_led_status)
{
	gpio_set_level(Green_led_pin, Green_led_status);
	gpio_set_level(Yellow_led_pin, Yellow_led_status);
	gpio_set_level(Red_led_pin, Red_led_status);
}

static void Battery_task(void *pvParameter)
{
	uint32_t ADC_voltage 	= 0;
	int ADC_reading 		= 0;
	int Battery_mV 		= 0;
	while(1)
	{
		//Leest de ADC en berekend de batterij spanning
		ADC_reading = adc1_get_raw((adc1_channel_t)Battery_voltage);
		ADC_voltage = esp_adc_cal_raw_to_voltage(ADC_reading, ADC_characteristics);
		Battery_mV = 2*ADC_voltage;
		//Als batterij spanning boven de 3.8V is, wordt de groene led aangestuurd
		if(Battery_mV >= 3800)
		{
			Control_led(1,0,0);
		}
		if((Battery_mV >= 3400) && (Battery_mV < 3800))
		{
			Control_led(0,1,0);
		}
		if((Battery_mV >= 3000) && (Battery_mV < 3400))
		{
			Control_led(0,0,1);
		}
		else
		{
			gpio_set_level(Power_on_off_pin, 0);
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

static void tp_example_touch_pad_init(void)
{
    touch_pad_config(TOUCH_PAD_NUM2, TOUCH_THRESH_NO_USE);
}

void app_main(void)
{
	Vibrator_motor_init();
    GPIO_init();
    ADC_init();
    Interrupt_init();
    tcp_client_init();
    start_measure_time = esp_timer_get_time();
	printf("ESP STARTED\n");
	xTaskCreate(&Battery_task, "Battery voltage measurement", 2048, NULL, 5, NULL);
	vTaskDelay(2500 / portTICK_PERIOD_MS);
	xTaskCreate(&Motor1_task, "Vibrator Motor 1", 2048, NULL, 5, NULL);
    xTaskCreate(&Resistive_stretch1_task, "Resistive stretch sensor 1", 2048, NULL, 5, NULL);
    xTaskCreate(&Resistive_stretch2_task, "Resistive stretch sensor 2", 2048, NULL, 5, NULL);
//    xTaskCreate(&tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
	uint8_t reg = 0x06;
	uint8_t data = 0x01;
	spi_device_handle_t spi;
	spi = SPI_init();
	IMU_write_reg(spi, reg, data); 				//Selecteerd de klok van de IMU
    IMU_read_ID(spi);							//Leest de ID van de IMU
    IMU_init_Magneto(spi);						//Initialiseert de magnetometer voor communicatie

//    touch_pad_init();
//    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V5);
//    touch_pad_set_cnt_mode(TOUCH_PAD_NUM2, TOUCH_PAD_SLOPE_1, TOUCH_PAD_TIE_OPT_LOW);
//    touch_pad_set_meas_time(0x0001, 0xffff);
//    tp_example_touch_pad_init();

}
