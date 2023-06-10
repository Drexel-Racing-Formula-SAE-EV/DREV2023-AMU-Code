/*
 * cli.h
 *
 *  Created on: May 2, 2023
 *      Author: kauff
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "help.h"
#include "BMS.h"
#include "driver_test.h"

static void cli_help(uint8_t nargs, char **args);
static void rx_byte(char cRxByte);
int cli_putc(const char str);
void cli_handle_command(char *_command);
int cli_puts(const char *str);
void cli_process_data(uint8_t _head, uint8_t _tail);

#endif /* INC_CLI_H_ */
