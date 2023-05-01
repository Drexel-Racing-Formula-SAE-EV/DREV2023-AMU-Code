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
//#include "ltc6813.h"
//#include "main.h" // need to move implicit declares in main.c/h to help.h!!

//DEFINE GPIO's
#define CSA4_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
#define CSA4_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

#define CSB7_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
#define CSB7_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
//SPI2_CS_1
#define CSB12_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#define CSB12_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
//SPI2_CS_2
#define CSB14_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
#define CSB14_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//SPI3_CS_1
#define CSB0_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
#define CSB0_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
//SPI3_CS_2
#define CSB1_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
#define CSB1_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

//FAN CONTROLLERS
#define E7_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
#define E7_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

#define E8_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
#define E8_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);

#define E9_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
#define E9_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);

#define E10_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
#define E10_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);

#define E11_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
#define E11_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

#define E12_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
#define E12_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

#define E13_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
#define E13_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

#define E14_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);
#define E14_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);

#define E15_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
#define E15_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);

#define D8_RESET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
#define D8_SET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);

#define D9_RESET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
#define D9_SET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);

#define D10_RESET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
#define D10_SET HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);

#define LTC_6813_CS_RESET CSB12_RESET
#define LTC_6813_CS_SET CSB12_SET

#define RXBUF_SIZE 40
#define TXBUF_SIZE 40


#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#define ARGS_COUNT_MAX 8

#define MSG_LEN 128

typedef struct
{
	CAN_HandleTypeDef *hcan1;

	SPI_HandleTypeDef *hspi1;
	SPI_HandleTypeDef *hspi2;

	UART_HandleTypeDef *huart2;

	TIM_HandleTypeDef *htim1;

	DMA_HandleTypeDef *hdma_usart2_rx;

	uint8_t debug;

	uint16_t v_max;

	uint16_t v_min;

	uint16_t v_avg;
} app_data;

/// Serial command callback function definition
typedef void (*sc_cmd_cb)(uint8_t nargs, char **args);

/// Serial command struct definition
typedef struct {
    char *cmdName;     ///< command name
    char *cmdDesc;     ///< command description
    sc_cmd_cb cbFunc;  ///< callback function
} sc_command;

typedef struct APP_CMD {
    char COMMAND[RXBUF_SIZE];
} APP_CMD_t;

void u_sleep(uint32_t microseconds);

void init_app_data_help(app_data *app_data_init);

void spi_infinite_send(uint8_t *stop_flag);

void spi_loopback(uint8_t *stop_flag);

#endif /* INC_HELP_H_ */
