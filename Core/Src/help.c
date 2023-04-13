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
	  //printf("here 1\r\n");
	  /* Go to number of cycles for system */
	  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
	  //printf("here 2\r\n");
	  /* Delay till end */
	  //printf("%d : %d\r\n",DWT->CYCCNT- clk_cycle_start,microseconds);
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

void spi_loopback(uint8_t *stop_flag){
	printf("\r\n spi loopback\r\n;");
	char spi_tx_buffer[200]={0};
	char spi_rx_buffer[200]={0};
	uint16_t spi_transfer_size = 200;
	for(int i = 0; i<200; i++){
	  spi_tx_buffer[i] = i;
	}
	while(*stop_flag){
	   	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	   	  HAL_SPI_TransmitReceive(a_d.hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
	   	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	   	  for(int i = 0; i<200; i++){
	   		  printf("%d ",spi_tx_buffer[i]);
	   	  }
	   	  for(int i = 0; i<200; i++){
	   		  printf("%d ",spi_rx_buffer[i]);
	   	  }
	         HAL_Delay(1000);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(1000);
  	printf("\r\n exiting test2\r\n;");

}

void spi_infinite_send(uint8_t *stop_flag){
	printf("\r\n entering test2\r\n;");
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  uint8_t blah[1] = {0x02};
	  while(*stop_flag){
			HAL_SPI_Transmit(a_d.hspi1, blah,1,100);
			//for(uint8_t i = 0; i<tx_len+rx_len; i++){
			  //	printf("%x\r\n",rx_data[i]);
			//  }
	         HAL_Delay(1000);
	  }
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(1000);
  	printf("\r\n exiting test2\r\n;");

}
