/*
 * BMS.h
 *
 *  Created on: May 2, 2023
 *      Author: kauff
 */

#ifndef INC_BMS_H_
#define INC_BMS_H_

#include "LTC681x.h"
#include "help.h"

void spi_infinite_send(uint8_t *stop_flag);

void spi_loopback(uint8_t *stop_flag);

void test1(void);

void spi_comm_test(void);

void test4(void);

void test5(void);

void coll_cell_volt(void);

void temp_calc(void);

void volt_calc(void);

void get_cell_data(void);

void can_test(void);

void pwm_out_test(void);

void pwm_in_test(void);

void dac_test(void);

void charging_mode();

void discharge_mode();

void print_conv_time(uint32_t conv_time);

void check_error(int error);

void print_cells(uint8_t datalog_en);

void print_rxcomm(void);

void print_wrcomm(void);

void init_app_data_bms(app_data *app_data_init);

#endif /* INC_BMS_H_ */
