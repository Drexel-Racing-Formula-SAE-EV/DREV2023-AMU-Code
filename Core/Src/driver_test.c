/*
 * driver_test.c
 *
 *  Created on: Jun 10, 2023
 *      Author: kauff
 */

#include "driver_test.h"

static app_data *a_d;

void spi_loopback(uint8_t nargs, char **args){
	printf("\r\n spi loopback\r\n;");
	char spi_tx_buffer[200]={0};
	char spi_rx_buffer[200]={0};
	uint16_t spi_transfer_size = 200;
	for(int i = 0; i<200; i++){
	  spi_tx_buffer[i] = i;
	}
	while(a_d->stop_flag){
	   	  LTC_6813_CS_RESET
	   	  HAL_SPI_TransmitReceive(a_d->hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
	   	  LTC_6813_CS_SET
	   	  for(int i = 0; i<200; i++){
	   		  printf("%d ",spi_tx_buffer[i]);
	   	  }
	   	  for(int i = 0; i<200; i++){
	   		  printf("%d ",spi_rx_buffer[i]);
	   	  }
	         HAL_Delay(1000);
	}
	LTC_6813_CS_SET
	HAL_Delay(1000);
  	printf("\r\n exiting test2\r\n;");

}

void spi_infinite_send(uint8_t nargs, char **args){
	printf("\r\n entering test2\r\n;");
	  LTC_6813_CS_RESET
	  uint8_t blah[1] = {0x02};
	  while(a_d->stop_flag){
			HAL_SPI_Transmit(a_d->hspi1, blah,1,100);
			//for(uint8_t i = 0; i<tx_len+rx_len; i++){
			  //	printf("%x\r\n",rx_data[i]);
			//  }
	         HAL_Delay(1000);
	  }
	  LTC_6813_CS_SET
      HAL_Delay(1000);
  	printf("\r\n exiting test2\r\n;");

}

void test1(uint8_t nargs, char **args){
	printf("\r\n entering test1\r\n;");
	  char spi_tx_buffer[200];
	  char spi_rx_buffer[200];
	  uint16_t spi_transfer_size = 200;
	  while (a_d->stop_flag)//stop_flag)
	  {
		  for(int i = 0; i<200; i++){
			  spi_tx_buffer[i] = i;
		  }
	         //printf("Hello World\n\r");//wont see until uart to usb recieved
	   	  LTC_6813_CS_RESET
	   	  HAL_SPI_TransmitReceive(a_d->hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
	   	//spi_write_read(spi_tx_buffer,200,spi_rx_buffer,200);
	   	  LTC_6813_CS_SET
	   	  for(int i = 0; i<200; i++){
	   		  printf("%d ",spi_rx_buffer[i]);
	   	  }
	         //turn off rgb leds
	         //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

	         HAL_Delay(1000);
		}
	  printf("\r\n exiting test1\r\n;");
}

void test5(uint8_t nargs, char **args){
	printf("starting sleep\r\n");
	for(int i=0;i<10;i++){
		LTC_6813_CS_RESET
		u_sleep(300);
		LTC_6813_CS_SET
		u_sleep(300);
	}

	printf("ending sleep\r\n");
}

void pwm_out_test(uint8_t nargs, char **args){
	  int32_t dutyCycle = 0;
	  while (1)
	  {
		   while(dutyCycle < 65535)
		        {
		            TIM1->CCR1 = dutyCycle;
		            TIM1->CCR2 = dutyCycle;
		            TIM1->CCR3 = dutyCycle;
		            TIM1->CCR4 = dutyCycle;
		            TIM3->CCR1 = dutyCycle;
					TIM3->CCR2 = dutyCycle;
					TIM3->CCR3 = dutyCycle;
					TIM3->CCR4 = dutyCycle;
					TIM4->CCR1 = dutyCycle;
					TIM4->CCR2 = dutyCycle;
					TIM4->CCR3 = dutyCycle;
					TIM4->CCR4 = dutyCycle;
		            dutyCycle += 100;
		            HAL_Delay(10);
		        }
		        while(dutyCycle > 0)
		        {
		            TIM1->CCR1 = dutyCycle;
		            TIM1->CCR2 = dutyCycle;
		            TIM1->CCR3 = dutyCycle;
		            TIM1->CCR4 = dutyCycle;
		            TIM3->CCR1 = dutyCycle;
					TIM3->CCR2 = dutyCycle;
					TIM3->CCR3 = dutyCycle;
					TIM3->CCR4 = dutyCycle;
					TIM4->CCR1 = dutyCycle;
					TIM4->CCR2 = dutyCycle;
					TIM4->CCR3 = dutyCycle;
					TIM4->CCR4 = dutyCycle;
		            dutyCycle -= 100;
		            HAL_Delay(10);
		        }
	  }
}

void pwm_in_test(uint8_t nargs, char **args){
	printf("Duty %f Freq %lu\r\n",a_d->Duty,a_d->Freq);
}

void dac_test(uint8_t nargs, char **args){
	uint32_t DAC_OUT[4] = {0, 1241, 2482, 3723};
	uint8_t i = 0;
	while(1){
        HAL_DAC_SetValue(a_d->hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_OUT[i++]);
        HAL_DAC_SetValue(a_d->hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_OUT[i++]);
        if(i == 4)
        {
            i = 0;
        }
        HAL_Delay(50);
	}
}

void can_test(uint8_t nargs, char **args){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	//uint8_t TxData[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	uint8_t TxData[3] = {0x90 ,0xAB, 0x2A};
    if (HAL_CAN_AddTxMessage(a_d->hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
    	printf("broke\r\n");
      //Error_Handler();
    }
}





void init_app_data_dt(app_data *app_data_init)
{
	a_d = app_data_init;
}
