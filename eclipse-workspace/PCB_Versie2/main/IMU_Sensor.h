/*
 * IMU_Sensor.h
 *
 *  Created on: 2 jun. 2021
 *      Author: Gebruiker
 */
#ifndef MAIN_IMU_SENSOR_H_
#define MAIN_IMU_SENSOR_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


#define IMU_NUM_CS   5


void IMU_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data);
void IMU_read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *data);
void IMU_read_ID(spi_device_handle_t spi);
void imu_cs_init(void);
void IMU_init_Magneto(spi_device_handle_t spi);
void IMU_read_data(spi_device_handle_t spi);

#endif /* MAIN_IMU_SENSOR_H_ */
