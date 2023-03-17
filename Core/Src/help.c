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
}
