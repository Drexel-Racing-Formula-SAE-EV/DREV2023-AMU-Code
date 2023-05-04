/*
 * BMS.c
 *
 *  Created on: May 2, 2023
 *      Author: kauff
 */

#include "BMS.h"

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0

#define CHARGING 1
#define DISCHARGING 2

static app_data a_d;

/********************************************************************
 ADC Command Configurations. See LTC681x.h for options(Reference Linduino LTC6813)
*********************************************************************/
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE =MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT =CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
const uint8_t SEL_REG_A = REG_1; //!< Register Selection
const uint8_t SEL_REG_B = REG_2; //!< Register Selection

int16_t CRC15_POLY = 0x4599;
uint8_t streg=0;
int8_t error = 0;
uint32_t conv_time = 0;
int8_t s_pin_read=0;

int mode_flag = 0;//flag for knowing when to stay in a mode and when to exit safely
const int vbat_max = 4.2;//set to the max voltage cars cell can charge to immediately ceases charging once exceeded
const int vbat_mix = 3.2;//minimum voltage cars cells can be upon hitting immediately shutdown
const int current_max = 2;//max current allowed for charging system
int CCL = 2; // current charge limit - initially the max current a cell can handle
int DCL = 2; // discharge current limit - initially the max current a cell can handle
int hall_current = 0;


void spi_loopback(uint8_t *stop_flag){
	printf("\r\n spi loopback\r\n;");
	char spi_tx_buffer[200]={0};
	char spi_rx_buffer[200]={0};
	uint16_t spi_transfer_size = 200;
	for(int i = 0; i<200; i++){
	  spi_tx_buffer[i] = i;
	}
	while(*stop_flag){
	   	  LTC_6813_CS_RESET
	   	  HAL_SPI_TransmitReceive(a_d.hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
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

void spi_infinite_send(uint8_t *stop_flag){
	printf("\r\n entering test2\r\n;");
	  LTC_6813_CS_RESET
	  uint8_t blah[1] = {0x02};
	  while(*stop_flag){
			HAL_SPI_Transmit(a_d.hspi1, blah,1,100);
			//for(uint8_t i = 0; i<tx_len+rx_len; i++){
			  //	printf("%x\r\n",rx_data[i]);
			//  }
	         HAL_Delay(1000);
	  }
	  LTC_6813_CS_SET
      HAL_Delay(1000);
  	printf("\r\n exiting test2\r\n;");

}

void test1(void){
	printf("\r\n entering test1\r\n;");
	  char spi_tx_buffer[200];
	  char spi_rx_buffer[200];
	  uint16_t spi_transfer_size = 200;
	  while (a_d.stop_flag)//stop_flag)
	  {
		  for(int i = 0; i<200; i++){
			  spi_tx_buffer[i] = i;
		  }
	         //printf("Hello World\n\r");//wont see until uart to usb recieved
	   	  LTC_6813_CS_RESET
	   	  HAL_SPI_TransmitReceive(a_d.hspi1, (uint8_t *) spi_tx_buffer,(uint8_t *) spi_rx_buffer,spi_transfer_size,100);
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

void spi_comm_test(void){
	printf("\r\n entering test3\r\n;");
	  //while(stop_flag){
	      for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
	      {
	        //Communication control bits and communication data bytes. Refer to the data sheet.
	        a_d.BMS_IC[current_ic].com.tx_data[0]= 0x81; // Icom CSBM Low(8) + data D0 (0x11)
	        a_d.BMS_IC[current_ic].com.tx_data[1]= 0x10; // Fcom CSBM Low(0)
	        a_d.BMS_IC[current_ic].com.tx_data[2]= 0xA2; // Icom CSBM Falling Edge (A) + data D1 (0x25)
	        a_d.BMS_IC[current_ic].com.tx_data[3]= 0x50; // Fcom CSBM Low(0)
	        a_d.BMS_IC[current_ic].com.tx_data[4]= 0xA1; // Icom CSBM Falling Edge (A) + data D2 (0x17)
	        a_d.BMS_IC[current_ic].com.tx_data[5]= 0x79; // Fcom CSBM High(9)
	      }
	      wakeup_sleep(TOTAL_IC);
	      LTC6813_wrcomm(TOTAL_IC,a_d.BMS_IC);
	      //print_wrcomm();

	      wakeup_idle(TOTAL_IC);
	      LTC6813_stcomm(3);

	      wakeup_idle(TOTAL_IC);
	      error = LTC6813_rdcomm(TOTAL_IC,a_d.BMS_IC);
	      check_error(error);
	      print_wrcomm();
	      print_rxcomm();
	      //HAL_Delay(1000);
	  //}
	  printf("\r\n exiting test3\r\n;");
}

void test4(void){
	printf("\r\n entering test4\r\n;");
	  //while (stop_flag)
	  //{
		  printf("starting cell voltage reading loop\r\n");
		  printf("recording wakeup sleep\r\n");
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
		  wakeup_sleep(TOTAL_IC);
		  //printf("pass1\r\n");
		  LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
		  //printf("pass2\r\n");
		  conv_time = LTC6813_pollAdc();
		  //printf("start ADC waiting 1 seconds\r\n");
		  //HAL_Delay(1000);

		  //printf("pass3\r\n");
		  print_conv_time(conv_time);
		  //printf("Cell Voltages:\r\n");

	      wakeup_sleep(TOTAL_IC);
	      error = LTC6813_rdcv(SEL_ALL_REG,TOTAL_IC,a_d.BMS_IC); // Set to read back all cell voltage registers
	      check_error(error);
	      print_cells(DATALOG_DISABLED);
	      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);
	      HAL_Delay(1000);
	      //turn off rgb leds
	      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
	      //HAL_Delay(1000);
	  //}
	  printf("\r\n exiting test4\r\n;");
}

void test5(void){
	printf("starting sleep\r\n");
	for(int i=0;i<10;i++){
		LTC_6813_CS_RESET
		u_sleep(300);
		LTC_6813_CS_SET
		u_sleep(300);
	}

	printf("ending sleep\r\n");
}

void volt_calc(void){//collects voltages across all ICs calculate minimum, maximum and avg voltage per segment
	uint16_t volt_min=65535,volt_max=0,volt_avg=0,total_cells=0;
	uint32_t volt_total=0;
	for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
	  {
	    for (int i=0; i<a_d.BMS_IC[0].ic_reg.cell_channels; i++){
	    	//printf("%d:%.4f\r\n",BMS_IC[current_ic].cells.c_codes[i],BMS_IC[current_ic].cells.c_codes[i]*0.0001);
	    	if(a_d.BMS_IC[current_ic].cells.c_codes[i]==65535||a_d.BMS_IC[current_ic].cells.c_codes[i]==0);
	    	else if(volt_min>a_d.BMS_IC[current_ic].cells.c_codes[i]){
	    		volt_min = a_d.BMS_IC[current_ic].cells.c_codes[i];
	    	}
	    	if(a_d.BMS_IC[current_ic].cells.c_codes[i]==65535||a_d.BMS_IC[current_ic].cells.c_codes[i]==0);
	    	else if(volt_max<a_d.BMS_IC[current_ic].cells.c_codes[i]){
	    		volt_max=a_d.BMS_IC[current_ic].cells.c_codes[i];
	    	}
	    	if(a_d.BMS_IC[current_ic].cells.c_codes[i]!=65535&&a_d.BMS_IC[current_ic].cells.c_codes[i]!=0){
				volt_total += a_d.BMS_IC[current_ic].cells.c_codes[i];
				total_cells++;
	    	}
	    }
	  }
    //printf("total cells: %d\r\n",total_cells);
    volt_avg = volt_total/total_cells;
    //printf("volt total %d, volt_avg %d\r\n",volt_total,volt_avg);
    //printf("vmax: %d, vmin %d, vavg, %d\r\n",volt_max,volt_min,volt_avg);
    a_d.v_max = volt_max;
    a_d.v_min = volt_min;
    a_d.v_avg = volt_avg;
}

void coll_cell_volt(void){
	  wakeup_sleep(TOTAL_IC);
	  LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
	  conv_time = LTC6813_pollAdc();
	  print_conv_time(conv_time);  //gotta fix this whole part

    wakeup_sleep(TOTAL_IC);
    error = LTC6813_rdcv(SEL_ALL_REG,TOTAL_IC,a_d.BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    print_cells(DATALOG_DISABLED);
}

void temp_calc(void){

}

void get_cell_data(void){
	printf("\r\nTotal Cells: %d\
			\r\nTotal IC:    %d\
			\r\nVolt Min:    %.04f\
			\r\nVolt Max:    %.04f\
			\r\nVolt Avg:    %.04f\r\n",0,TOTAL_IC,*a_d.v_min*.0001,*a_d.v_max*.0001,*a_d.v_avg*.0001);
}

void can_test(void){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	//uint8_t TxData[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
	uint8_t TxData[3] = {0x90 ,0xAB, 0x2A};
    if (HAL_CAN_AddTxMessage(a_d.hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
      /* Transmission request Error */
    	printf("broke\r\n");
      Error_Handler();
    }
}

void pwm_out_test(void){
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

void pwm_in_test(void){
	printf("Duty %f Freq %ul\r\n",*a_d.Duty,*a_d.Freq);
}

void dac_test(void){
	uint32_t DAC_OUT[4] = {0, 1241, 2482, 3723};
	uint8_t i = 0;
	while(1){
        HAL_DAC_SetValue(a_d.hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_OUT[i++]);
        HAL_DAC_SetValue(a_d.hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_OUT[i++]);
        if(i == 4)
        {
            i = 0;
        }
        HAL_Delay(50);
	}
}

void charging_mode(){//activated by GPIO Signal going high from external source(interrupt)
	while(mode_flag==CHARGING){
		//check if charge current limit >0
		if(CCL>0){
			//ADC of hall effect-at pin PC1
			if(hall_current>=CCL+2){//check if pack current >= charge current limit+xAmps(x=us defined threshold)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off charging

			}
			else{
				//allow charging to continue - repeats the loop
			}
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off power input
		}
	}
}

void discharge_mode(){//default mode ~when GPIO Signal is low
	while(mode_flag==DISCHARGING){
		//check if discharge current limit >0
		if(DCL>0){
			if(hall_current < DCL+2){//check if pack current >= discharge current limit+xAmps(x=us defined threshold)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off power output
			}
			else{

			}
		}
		else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//turn off power output
		}
	}
}

void print_conv_time(uint32_t conv_time)
{
  uint16_t m_factor=1000;  // to print in ms

  //Serial.print(F("Conversion completed in:"));
  //Serial.print(((float)conv_time/m_factor), 1);
  //Serial.println(F("ms \n"));
  printf("Conversion completed in %f ms\r\n",(float)conv_time/m_factor);
}

void check_error(int error)
{
  if (error == -1)
  {
    printf("A PEC error was detected in the received data\r\n");
  }
}

void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      //Serial.print(" IC ");
      //Serial.print(current_ic+1,DEC);
      //Serial.print(", ");
      printf(" IC %d,",current_ic+1);
      for (int i=0; i<a_d.BMS_IC[0].ic_reg.cell_channels; i++)
      {
        //Serial.print(" C");
        //Serial.print(i+1,DEC);
        //Serial.print(":");
        //Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        //Serial.print(",");
        printf(" C%d:%.4f,",i+1,a_d.BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
      //Serial.println();
      printf("\r\n");
    }
    else
    {
      //Serial.print(" Cells, ");
      printf(" Cells, ");
      for (int i=0; i<a_d.BMS_IC[0].ic_reg.cell_channels; i++)
      {
        //Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        //Serial.print(",");
        printf("%f,",a_d.BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
    }
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************************
  \brief prints data which is written on COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrcomm(void)
{
 int comm_pec;

  //Serial.println(F("Written Data in COMM Register: "));
  printf("Written Data in COMM Register: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC- "));
    //Serial.print(current_ic+1,DEC);
    printf(" IC- %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
      printf(", %x",a_d.BMS_IC[current_ic].com.tx_data[i]);
    }
    //Serial.print(F(", Calculated PEC: 0x"));
    printf(", Calculated PEC: ");
    comm_pec = pec15_calc(6,&a_d.BMS_IC[current_ic].com.tx_data[0]);
    //serial_print_hex((uint8_t)(comm_pec>>8));
    printf("%x",comm_pec>>8);
    //Serial.print(F(", 0x"));
    //serial_print_hex((uint8_t)(comm_pec));
    printf(", %x\r\n",comm_pec);
    //Serial.println("\n");
  }
}

/*!****************************************************************************
  \brief Prints received data from COMM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxcomm(void)
{
   printf("Received Data in COMM register: \r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC- "));
    //Serial.print(current_ic+1,DEC);
    printf(" IC- %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
      printf(", %x",a_d.BMS_IC[current_ic].com.rx_data[i]);
    }
    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %x, %x\r\n",a_d.BMS_IC[current_ic].com.rx_data[6],a_d.BMS_IC[current_ic].com.rx_data[7]);
  }
}

void init_app_data_bms(app_data *app_data_init)
{
	a_d = *app_data_init;
}

