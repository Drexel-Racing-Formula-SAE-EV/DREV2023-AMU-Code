/*
 * help.c
 *
 *  Created on: Mar 15, 2023
 *      Author: kauff
 */

#include "help.h"

app_data a_d;

void u_sleep (uint32_t microseconds)
{
	//printf("broken\r\n");
	//__HAL_TIM_SET_COUNTER(a_d.htim1,0);  // set the counter value a 0
	//printf("broken\r\n");
	//while (__HAL_TIM_GET_COUNTER(a_d.htim1) < us){
	//	printf("%d>%d\r\n",us,__HAL_TIM_GET_COUNTER(a_d.htim1));  // wait for the counter to reach the us input in the parameter
	//};
	//HAL_Delay();
	  uint32_t clk_cycle_start = DWT->CYCCNT;

	  /* Go to number of cycles for system */
	  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

	  /* Delay till end */
	  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void init_app_data_help(app_data *app_data_init)
{
	a_d = *app_data_init;
	if(a_d.debug==1){
		printf("\r\nDebugging init_app_data_help\r\n");
		uint8_t data[3],sent[3];
		sent[0]=0;
		while (sent[0]<3){
			sent[0] +=1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(a_d.hspi1, (uint8_t *) sent,(uint8_t *) data,1,100);
			printf("data sent %d :: data in init: %d \r\n",sent[0],data[0]);
			HAL_SPI_TransmitReceive(app_data_init->hspi1, (uint8_t *) sent,(uint8_t *) data,1,100);
			printf("data sent %d :: data in init: %d \r\n",sent[0],data[0]);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_Delay(1000);
		}
	}
}
