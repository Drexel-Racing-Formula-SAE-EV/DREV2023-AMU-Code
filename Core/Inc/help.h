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
//#include "LTC681x.h"
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
#define CSE4_RESET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
#define CSE4_SET HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
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

#define MSG_LEN 256//default 128

#define TOTAL_IC 5//should be 5

/*! Cell Voltage data structure. */
typedef struct
{
  uint16_t c_codes[18]; //!< Cell Voltage Codes
  uint8_t pec_match[6]; //!< If a PEC error was detected during most recent read cmd
} cv;

/*! AUX Reg Voltage Data structure */
typedef struct
{
  uint16_t a_codes[9]; //!< Aux Voltage Codes
  uint8_t pec_match[4]; //!< If a PEC error was detected during most recent read cmd
} ax;

/*! Status Reg data structure. */
typedef struct
{
  uint16_t stat_codes[4]; //!< Status codes.
  uint8_t flags[3]; //!< Byte array that contains the uv/ov flag data
  uint8_t mux_fail[1]; //!< Mux self test status flag
  uint8_t thsd[1]; //!< Thermal shutdown status
  uint8_t pec_match[2]; //!< If a PEC error was detected during most recent read cmd
} st;

/*! IC register structure. */
typedef struct
{
  uint8_t tx_data[6];  //!< Stores data to be transmitted
  uint8_t rx_data[8];  //!< Stores received data
  uint8_t rx_pec_match; //!< If a PEC error was detected during most recent read cmd
} ic_register;

/*! PEC error counter structure. */
typedef struct
{
  uint16_t pec_count; //!< Overall PEC error count
  uint16_t cfgr_pec;  //!< Configuration register data PEC error count
  uint16_t cell_pec[6]; //!< Cell voltage register data PEC error count
  uint16_t aux_pec[4];  //!< Aux register data PEC error count
  uint16_t stat_pec[2]; //!< Status register data PEC error count
} pec_counter;

/*! Register configuration structure */
typedef struct
{
  uint8_t cell_channels; //!< Number of Cell channels
  uint8_t stat_channels; //!< Number of Stat channels
  uint8_t aux_channels;  //!< Number of Aux channels
  uint8_t num_cv_reg;    //!< Number of Cell voltage register
  uint8_t num_gpio_reg;  //!< Number of Aux register
  uint8_t num_stat_reg;  //!< Number of  Status register
} register_cfg;

/*! Cell variable structure */
typedef struct
{
  ic_register config;
  ic_register configb;
  cv  cells;
  ax  aux;
  st  stat;
  ic_register com;
  ic_register pwm;
  ic_register pwmb;
  ic_register sctrl;
  ic_register sctrlb;
  uint8_t sid[6];
  uint8_t isospi_reverse;
  pec_counter crc_count;
  register_cfg ic_reg;
  long system_open_wire;
} cell_asic;

typedef struct
{
	uint16_t OV_THRESHOLD;
	uint16_t UV_THRESHOLD;
	uint16_t MEASUREMENT_LOOP_TIME;
	uint8_t REFON;
	uint8_t ADCOPT;
	uint8_t GPIOBITS_A[5];
	uint8_t GPIOBITS_B[4];
	uint16_t UV;
	uint16_t OV;
	uint8_t DCCBITS_A[12];
	uint8_t DCCBITS_B[7];
	uint8_t DCTOBITS[4];
	uint8_t FDRF;
	uint8_t DTMEN;
	uint8_t PSBITS[2];

}ltcvar;

typedef struct{
	 uint16_t v_max;
	 uint16_t v_min;
	 uint16_t v_avg;

	//cell voltage not balanced/too high array[18]
	 uint8_t cvnb[18];
	//track cvnb position for marking unbalanced cells
	 uint8_t tap;
	 uint8_t old_tap;
}segment;

typedef struct
{
	CAN_HandleTypeDef *hcan1;

	SPI_HandleTypeDef *hspi1;
	SPI_HandleTypeDef *hspi2;

	UART_HandleTypeDef *huart2;

	TIM_HandleTypeDef *htim1;
	TIM_HandleTypeDef *htim3;
	TIM_HandleTypeDef *htim4;
	TIM_HandleTypeDef *htim8;
	TIM_HandleTypeDef *htim9;

	DMA_HandleTypeDef *hdma_usart2_rx;

	DAC_HandleTypeDef *hdac;

	cell_asic *BMS_IC;

	segment seg[5];

	ltcvar ltc;

	 uint8_t stop_flag;

	//PWM IN Values
	 uint32_t Freq;
	 float Duty;

	 uint8_t debug;

	//BALANCING
	//sets the discharge pin
	 uint8_t s_pin;

	//USER PARAMETERS
	 float max_curr;
	 float max_cell_volt;
	 float min_cell_volt;
	 float max_sys_volt;
	 float min_sys_volt;
	 uint8_t max_soc;
	 uint8_t min_soc;

	 uint8_t VDisp;

	 uint8_t mode;

	 volatile float hall_current;

	 //set these flags to be only 1 bit somehow...
	 volatile uint8_t temp_safe;
	 volatile uint8_t volt_safe;
	 volatile uint8_t curr_safe;


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

#endif /* INC_HELP_H_ */
