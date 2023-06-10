/*
 * BMS_print.h
 *
 *  Created on: Jun 10, 2023
 *      Author: kauff
 */

#ifndef INC_BMS_PRINT_H_
#define INC_BMS_PRINT_H_

#include "help.h"
#include "LTC681x.h"

void print_cells(uint8_t datalog_en);

void print_wrconfig(void);

void print_wrconfigb(void);

void print_rxconfig(void);

void print_rxconfigb(void);

void print_aux(uint8_t datalog_en);

void print_stat(void);

void print_aux1(uint8_t datalog_en);

void print_sumofcells(void);

void check_mux_fail(void);

void print_selftest_errors(uint8_t adc_reg ,int8_t error);

void print_overlap_results(int8_t error);

void print_digital_redundancy_errors(uint8_t adc_reg ,int8_t error);

void print_open_wires(void);

void print_pec_error_count(void);

void print_wrpwm(void);

void print_rxpwm(void);

void print_wrsctrl(void);

void print_rxsctrl(void);

void print_wrpsb(uint8_t type);

void print_rxpsb(uint8_t type);

void print_rxcomm(void);

void print_wrcomm(void);

void init_app_data_bmsp(app_data *app_data_init);

#endif /* INC_BMS_PRINT_H_ */
