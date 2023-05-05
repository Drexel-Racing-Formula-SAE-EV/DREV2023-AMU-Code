/*
 * cli.c
 *
 *  Created on: May 2, 2023
 *      Author: kauff
 */
#include "cli.h"

//uart cli
extern uint8_t cli_msg_pending;
uint8_t command_len;
uint16_t cyp_cmd_tail;
uint16_t cyp_cmd_head;
extern uint8_t command_buf[RXBUF_SIZE];
extern uint8_t rx_buf[RXBUF_SIZE];
extern uint8_t tx_buf[TXBUF_SIZE];

extern UART_HandleTypeDef huart2;

extern osMutexId_t uart_mutexHandle;

static const char *MSG_HELP_START
    = "<-- Help Menu: CMD Name - Description -->\r\n";
static const char *MSG_HELP_END
    = "<--------------------------------------->\r\n";
static const char *ERR_ARG_COUNT
    = "nERR: Expected %d arguments but received %d\r\n";
static const char *ERR_BAD_COMMAND
    = "ERR: Unrecognized command. Type \"help\" for help\r\n";

static const sc_command console_commands[] = {
    {"help",    "Displays list of all available CLI commands",  cli_help          },
	{"display", "Displays various options Paramaters : ", },
	{"1", "Runs Test 1", test1},
	{"spi_comm", "Spi Comm Test",spi_comm_test },
	{"4", "Runs Test 4",test4 },
	{"5", "Runs Test 5",test5 },
	{"charge", "Turns On Charging Mode",charging_mode },
	{"discharge", "Turns of Discharge Mode",discharge_mode },
	{"sis", "Infinite SPI",spi_infinite_send },
	{"spil", "Spi Loopback", spi_loopback},
	{"dcd", "Displays Cell Data", get_cell_data},
	{"ccv", "Collects Cell Voltage", coll_cell_volt},
	{"vc", "Calculates Voltage", volt_calc},
	{"cant", "Can Test", can_test},
	{"pwmot", "PWM out Test WARNING INFINITE", pwm_out_test},
	{"pwmint", "PWM in Test WARNING INFINITE", pwm_in_test},
	{"dact", "DAC TEST WARNING INFINITE", dac_test},
	{"cbt", "Cell Balancing Test", cb_test},
	{"sb", "Stops All Cell Balancing", stop_balance}
};

uint8_t m2mOn = 0;

char err_msg[MSG_LEN];
char prnt_msg[MSG_LEN];


uint8_t get_cli_msg_pending() {
    return cli_msg_pending;
}

void cli_handle_command(char *_command) {
    uint8_t cmd_found = 0;
    uint16_t nargs = 0;
    char *ptr;
    char *args[ARGS_COUNT_MAX];
    // Parse arguemtns (space delimited)
    args[nargs++] = _command;
    ptr           = strpbrk(_command, " ");
    while(ptr != NULL) {
        args[nargs] = ptr + 1;
        *ptr        = '\0';
        ptr         = strpbrk(args[nargs], " ");
        nargs++;
        if(nargs == ARGS_COUNT_MAX) {
            break;
        }
    }
    // Print newline after msg
    cli_puts("\r\n");
    // Iterate over list of console comands
    for(int i = 0; i < COUNTOF(console_commands); i++) {
        if(strcmp(_command, console_commands[i].cmdName) == 0) {
            console_commands[i].cbFunc(nargs - 1, args);
            cmd_found = 1;
            break;
        }
    }

    if(cmd_found == 0 && strlen(_command) > 0) {
        snprintf(err_msg, MSG_LEN, ERR_BAD_COMMAND);
        cli_puts(err_msg);
    }
}

int cli_puts(const char *str) {
    int ret = -1;
    if(osMutexAcquire(uart_mutexHandle, 1000) == osOK) {
        ret = 0;
        for(int i = 0; i < strlen(str); i++) {
            if(putchar(str[i]) != str[i]) {
                ret = -1;
            }
        }
        osMutexRelease(uart_mutexHandle);
    }
    return ret;
}

int cli_putc(const char str) {
    int ret = -1;
    if(osMutexAcquire(uart_mutexHandle, 1000) == osOK) {
        ret = 0;
            if(putchar(str) != str) {
                ret = -1;
                printf("failed\r\n");
            }
        osMutexRelease(uart_mutexHandle);
    }
    return ret;
}

static void cli_help(uint8_t nargs, char **args) {
    // Iterate over list of console commands
    cli_puts(MSG_HELP_START);
    for(int i = 0; i < COUNTOF(console_commands); i++) {
        snprintf(prnt_msg,
                 MSG_LEN,
                 "%s - %s\r\n",
                 console_commands[i].cmdName,
                 console_commands[i].cmdDesc);
        cli_puts(prnt_msg);
    }
    cli_puts(MSG_HELP_END);
}

void cli_process_data(uint8_t _head, uint8_t _tail) {
    // If circular buffer does not wrap around
    if(_head > _tail) {
        while(_tail < _head) {
            rx_byte(rx_buf[_tail]);
            _tail++;
        }
    }
    // If circular buffer wraps around
    else if(_head < _tail) {
        // Read to end of buffer
        while(_tail < RXBUF_SIZE) {
            rx_byte(rx_buf[_tail]);
            _tail++;
        }
        // Read from front of buffer to head
        _tail = 0;
        while(_tail < _head) {
            rx_byte(rx_buf[_tail]);
            _tail++;
        }
    }
}
uint8_t space =' ';
static void rx_byte(char cRxByte) {
    // End of message = Return carriage '\r'
    if(cRxByte == '\r') {
        cli_puts("\r\n");
        command_len = 0;
        cli_msg_pending = 1;
    }
    // If the received character is delete or backspace
    else if(cRxByte == 0x7f || cRxByte == 0x08) {
        // Decrement index and clear out previous character
        command_len--;
        command_buf[command_len] = 0x00;
        if(m2mOn == 0) {
           // putchar(cRxByte);
            //printf("%c",cRxByte);
            //cli_putc(cRxByte);
            HAL_UART_Transmit(&huart2,&cRxByte,1,500);
            HAL_UART_Transmit(&huart2,&space,1,500);
            HAL_UART_Transmit(&huart2,&cRxByte,1,500);
        }
    }
    // Print newline char to terminal
    else if(cRxByte == '\n') {
        if(m2mOn == 0) {
           // putchar(cRxByte);
            //printf("%c",cRxByte);
        	 cli_putc(cRxByte);
        }
    }
    // Not end of message; keep appending
    else {
        command_buf[command_len] = cRxByte;
        command_len++;
        if(m2mOn == 0) {
           // putchar(cRxByte);
            //printf("%c\r\n",cRxByte);
            HAL_UART_Transmit(&huart2,&cRxByte,1,500);
        	 //cli_putc(cRxByte);

        	 //printf("inside \r\n");
        }
    }
}

