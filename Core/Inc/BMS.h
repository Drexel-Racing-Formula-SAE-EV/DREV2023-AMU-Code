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
#include "BMS_print.h"

void spi_comm_test(uint8_t nargs, char **args);

void test4(uint8_t nargs, char **args);

void run_test(uint8_t nargs, char **args);

void display(uint8_t nargs, char **args);

void edit_params(uint8_t nargs, char **args);

void chg_mode(uint8_t nargs, char **args);

void coll_cell_volt(void);

void temp_calc(uint8_t channel);

void temp_test(void);

void volt_calc(uint8_t nargs, char **args);

void cb_test(void);

void stop_balance(uint8_t nargs, char **args);

void bal_all(uint8_t nargs, char **args);

void bal_cell(uint8_t segment);

void get_cell_data(uint8_t nargs, char **args);

void coll_unbalanced_cells(void);

void fan_control(uint8_t nargs, char **args);

void charging_mode();

void discharge_mode();

void balancing_mode();

void vmon();

void shutdown();

void print_conv_time(uint32_t conv_time);

void check_error(int error);

void init_app_data_bms(app_data *app_data_init);

#endif /* INC_BMS_H_ */
