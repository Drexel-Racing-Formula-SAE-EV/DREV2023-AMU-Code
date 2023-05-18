/*
 * BMS.h
 *
 *  Created on: May 2, 2023
 *      Author: kauff
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

#include "LTC681x.h"
#include "LTC6813.h"
#include "help.h"
#include "string.h"
#include "cmsis_os.h"


void spi_infinite_send(uint8_t nargs, char **args);

void spi_loopback(uint8_t nargs, char **args);

void test1(uint8_t nargs, char **args);

void spi_comm_test(uint8_t nargs, char **args);

void test4(uint8_t nargs, char **args);

void test5(uint8_t nargs, char **args);

void run_test(uint8_t nargs, char **args);

void display(uint8_t nargs, char **args);

void edit_params(uint8_t nargs, char **args);

void chg_mode(uint8_t nargs, char **args);

void coll_cell_volt(void);

void temp_calc(uint8_t nargs, char **args);

void volt_calc(uint8_t nargs, char **args);

void cb_test(void);

void stop_balance(uint8_t nargs, char **args);

void bal_all(uint8_t nargs, char **args);

void bal_cell(uint8_t segment);

void get_cell_data(uint8_t nargs, char **args);

void coll_unbalanced_cells(void);

void fan_control(uint8_t nargs, char **args);

void can_test(uint8_t nargs, char **args);

void pwm_out_test(uint8_t nargs, char **args);

void pwm_in_test(uint8_t nargs, char **args);

void dac_test(uint8_t nargs, char **args);

void charging_mode();

void discharge_mode();

void vmon();

void print_conv_time(uint32_t conv_time);

void check_error(int error);

void print_cells(uint8_t datalog_en);

void print_wrconfig(void);

void print_wrconfigb(void);

void print_rxconfig(void);

void print_rxconfigb(void);



void print_rxcomm(void);

void print_wrcomm(void);

void init_app_data_bms(app_data *app_data_init);

#endif /* INC_BMS_H_ */
