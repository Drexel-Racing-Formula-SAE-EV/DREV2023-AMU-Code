/*
 * driver_test.h
 *
 *  Created on: Jun 10, 2023
 *      Author: kauff
 */

#ifndef INC_DRIVER_TEST_H_
#define INC_DRIVER_TEST_H_

#include "help.h"

void spi_infinite_send(uint8_t nargs, char **args);

void spi_loopback(uint8_t nargs, char **args);

void test1(uint8_t nargs, char **args);

void test5(uint8_t nargs, char **args);

void pwm_out_test(uint8_t nargs, char **args);

void pwm_in_test(uint8_t nargs, char **args);

void dac_test(uint8_t nargs, char **args);

void can_test(uint8_t nargs, char **args);

void init_app_data_dt(app_data *app_data_init);

#endif /* INC_DRIVER_TEST_H_ */
