/*
 * help.h
 *
 *  Created on: Mar 15, 2023
 *      Author: kauff
 */

#ifndef INC_HELP_H_
#define INC_HELP_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

typedef struct
{
	CAN_HandleTypeDef *hcan1;

	SPI_HandleTypeDef *hspi1;
	SPI_HandleTypeDef *hspi2;

	UART_HandleTypeDef *huart2;

	TIM_HandleTypeDef *htim1;

	uint8_t debug;
} app_data;

void u_sleep(uint32_t microseconds);

void init_app_data_help(app_data *app_data_init);

void spi_infinite_send(uint8_t *stop_flag);

void spi_loopback(uint8_t *stop_flag);

#endif /* INC_HELP_H_ */
