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

static app_data *a_d;

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

void spi_comm_test(uint8_t nargs, char **args){
	printf("\r\n entering test3\r\n;");
	  //while(stop_flag){
	      for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
	      {
	        //Communication control bits and communication data bytes. Refer to the data sheet.
	        a_d->BMS_IC[current_ic].com.tx_data[0]= 0x81; // Icom CSBM Low(8) + data D0 (0x11)
	        a_d->BMS_IC[current_ic].com.tx_data[1]= 0x10; // Fcom CSBM Low(0)
	        a_d->BMS_IC[current_ic].com.tx_data[2]= 0xA2; // Icom CSBM Falling Edge (A) + data D1 (0x25)
	        a_d->BMS_IC[current_ic].com.tx_data[3]= 0x50; // Fcom CSBM Low(0)
	        a_d->BMS_IC[current_ic].com.tx_data[4]= 0xA1; // Icom CSBM Falling Edge (A) + data D2 (0x17)
	        a_d->BMS_IC[current_ic].com.tx_data[5]= 0x79; // Fcom CSBM High(9)
	      }
	      wakeup_sleep(TOTAL_IC);
	      LTC6813_wrcomm(TOTAL_IC,a_d->BMS_IC);
	      //print_wrcomm();

	      wakeup_idle(TOTAL_IC);
	      LTC6813_stcomm(3);

	      wakeup_idle(TOTAL_IC);
	      error = LTC6813_rdcomm(TOTAL_IC,a_d->BMS_IC);
	      check_error(error);
	      print_wrcomm();
	      print_rxcomm();
	      //HAL_Delay(1000);
	  //}
	  printf("\r\n exiting test3\r\n;");
}

void test4(uint8_t nargs, char **args){
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
	      error = LTC6813_rdcv(SEL_ALL_REG,TOTAL_IC,a_d->BMS_IC); // Set to read back all cell voltage registers
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

void run_test(uint8_t nargs, char **args){
	if(nargs == 1){
		if(strcmp(args[1], "charge") == 0){
			charging_mode(0,NULL);
		}
		else if(strcmp(args[1], "discharge") == 0){
			discharge_mode(0,NULL);
		}
		else if(strcmp(args[1], "balance") == 0){
			//mode = balance;
			bal_all(0,NULL);
		}
	}
}

void display(uint8_t nargs, char **args){
	//HAL_Delay(50);//consider changing hal delay to os delay
	if(nargs == 1){
		if(strcmp(args[1], "overall") == 0){
//Displays parameters of the battery: max temp of each segment, overall voltage, SoC, instantaneous current, state of AIR control, BMS safe / unsafe, calculated isolation from IMD
			printf("	Temp of segment:\r\n");
			for(int i = 0; i<TOTAL_IC;i++){
				printf("		%d: %f",i+1,10.0);
			}
			printf("\r\n	Overall Volt\r\n");
			printf("	Soc %d\r\n",10);
			printf("	Current: %f\r\n",10.0);
			printf("	AIR Control: %d\r\n",0);
			if(1){
				printf("	BMS Safe\r\n");
			}
			else{
				printf("	BMS Unsafe\r\n");
			}
			printf("	IMD Isolation: %f\r\n",10.0);

		}
		else if(strcmp(args[1], "temp") == 0){
//Displays max temp of each segment, body temp values for each segment, overall temp of the pack
			printf("Temp of segment:\r\n");
			for(int i = 0; i<TOTAL_IC;i++){
				printf("		%d: %f",i+1,10.0);
			}
		}
		else if(strcmp(args[1], "volt") == 0){
//Display array of all voltages and overall voltage, updating 1/s
			a_d->VDisp = !a_d->VDisp;
			//vmon();
			/*for(int i =0;i<3;i++){
				print_cells(0);
				osDelay(1000);
			}*/
			//HAL_Delay(1000);
			//printf("VDISP = %d\r\n",a_d->VDisp);
			return;
		}
		else{
			printf("Incorrect ARG\r\n");
		}
	}
	else{
		printf("ARGS != 1\r\n");
	}
}

void edit_params(uint8_t nargs, char **args){
	if(nargs == 2){
		if(atoi(args[2])==0){
			printf("incorrect 2nd argument input\r\n");
		}
		else{
			if(strcmp(args[1], "MC") == 0){
				a_d->max_curr = atoi(args[2]);
				//printf("value is now %f",*a_d->max_curr);
			}
			else if(strcmp(args[1], "MACV") == 0){
				if(0<atoi(args[2])&&atoi(args[2])<6.6){
					a_d->max_cell_volt = atoi(args[2]);
				}
				//printf("value is now %f",*a_d->max_cell_volt);

			}
			else if(strcmp(args[1], "MICV") == 0){
				a_d->min_cell_volt = atof(args[2]);
				//printf("value is now %f",*a_d->min_cell_volt);
			}
			else if(strcmp(args[1], "MAOV") == 0){
				a_d->max_sys_volt = atof(args[2]);
				//printf("value is now %f",*a_d->max_sys_volt);
			}
			else if(strcmp(args[1], "MIOV") == 0){
				a_d->min_sys_volt = atof(args[2]);
				//printf("value is now %f",*a_d->min_sys_volt);
			}
			else if(strcmp(args[1], "MASOC") == 0){
				a_d->max_soc = atoi(args[2]);
				//printf("value is now %f",*a_d->max_soc);
			}
			else if(strcmp(args[1], "MISOC") == 0){
				a_d->min_soc = atoi(args[2]);
				//printf("value is now %f",*a_d->min_soc);
			}
			else if(strcmp(args[1], "SPIN") == 0){
				if(0<atoi(args[2])&&atoi(args[2])<18){
					a_d->s_pin = atoi(args[2]);
				}
				printf("value is now %d\r\n",a_d->s_pin);
			}
			else{
				printf("Incorrect ARG 1 Input\r\n");
			}
		}
	}
	else{
		printf("Incorrect number of ARGS should be 2\r\n");
	}
}

void chg_mode(uint8_t nargs, char **args){
	if(nargs == 1){
		if(strcmp(args[1], "charge") == 0){
			charging_mode(0,NULL);
		}
		else if(strcmp(args[1], "discharge") == 0){
			discharge_mode(0,NULL);
		}
		else if(strcmp(args[1], "balance") == 0){
			//mode = balance;
			bal_all(0,NULL);
		}
		else if(strcmp(args[1], "no") == 0){
			a_d->mode = 127;
		}
		else{
			printf("Incorrect Arg\r\n");
		}
	}
	else{
		printf("too few/many arguments\r\n");
	}
}

void volt_calc(uint8_t nargs, char **args){//collects voltages across all ICs calculate minimum, maximum and avg voltage per segment
	uint16_t volt_min=65535,volt_max=0,volt_avg=0,total_cells=0;
	uint32_t volt_total=0;
	for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
	  {
		volt_min=65535;volt_max=0;volt_avg=0;total_cells=0;
	    for (int i=0; i<a_d->BMS_IC[current_ic].ic_reg.cell_channels; i++){
	    	//printf("%d:%.4f\r\n",BMS_IC[current_ic].cells.c_codes[i],BMS_IC[current_ic].cells.c_codes[i]*0.0001);
	    	if(a_d->BMS_IC[current_ic].cells.c_codes[i]==65535||a_d->BMS_IC[current_ic].cells.c_codes[i]==0);
	    	else if(volt_min>a_d->BMS_IC[current_ic].cells.c_codes[i]){
	    		volt_min = a_d->BMS_IC[current_ic].cells.c_codes[i];
	    	}
	    	if(a_d->BMS_IC[current_ic].cells.c_codes[i]==65535||a_d->BMS_IC[current_ic].cells.c_codes[i]==0);
	    	else if(volt_max<a_d->BMS_IC[current_ic].cells.c_codes[i]){
	    		volt_max=a_d->BMS_IC[current_ic].cells.c_codes[i];
	    	}
	    	if(a_d->BMS_IC[current_ic].cells.c_codes[i]!=65535&&a_d->BMS_IC[current_ic].cells.c_codes[i]!=0){
				volt_total += a_d->BMS_IC[current_ic].cells.c_codes[i];
				total_cells++;
	    	}
	    }
	    a_d->seg[current_ic].v_max = volt_max;
		a_d->seg[current_ic].v_min = volt_min;
	    volt_avg = volt_total/total_cells;
		a_d->seg[current_ic].v_avg = volt_avg;
	  }
    //printf("total cells: %d\r\n",total_cells);
    //printf("volt total %d, volt_avg %d\r\n",volt_total,volt_avg);
    //printf("vmax: %d, vmin %d, vavg, %d\r\n",volt_max,volt_min,volt_avg);
}

//(uint8_t nargs, char **args) is causing everything to break... no clue why
void coll_cell_volt(void){//uint8_t nargs, char **args){
	wakeup_sleep(TOTAL_IC);
	//HAL_Delay(500);
	LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
	conv_time = LTC6813_pollAdc();
	//print_conv_time(conv_time);  //gotta fix this whole part

    wakeup_sleep(TOTAL_IC);
    error = LTC6813_rdcv(SEL_ALL_REG,TOTAL_IC,a_d->BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    if(a_d->VDisp == 1){
    	print_cells(DATALOG_DISABLED);
    }
}

void cb_test(void){//uint8_t nargs, char **args){
	/*if(nargs == 0){
	    s_pin_read = *a_d->s_pin;
	}
	else if(nargs == 1){
		s_pin_read = atoi(args[1]);
	}
	else{
		printf("too many arguments\r\n");
	}*/
	s_pin_read = a_d->s_pin;
    //s_pin_read = select_s_pin();
    //s_pin_read = 4;
    wakeup_sleep(TOTAL_IC);
    LTC6813_set_discharge(s_pin_read,TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfg(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC,a_d->BMS_IC);
    print_wrconfig();
    print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    print_rxconfig();
    print_rxconfigb();
}

void stop_balance(uint8_t nargs, char **args){
    wakeup_sleep(TOTAL_IC);
    LTC6813_clear_discharge(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfg(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC,a_d->BMS_IC);
    print_wrconfig();
    print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    print_rxconfig();
    print_rxconfigb();
    printf("balance stopped\r\n");
}

/*
 * Collects all voltages and balances every cell in segment to lowest voltage
 * -not sure how to implement with multiple segments
 */
void bal_all(uint8_t nargs, char **args){
	//tracks taps previous position
	uint8_t old_tap = 0;
	for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
		a_d->seg[curr_ic].old_tap = 0;
	}
	//was balancing stopped? 0 yes 1 no
	uint8_t stp = 0;
	printf("Starting Balancing\r\n");
	coll_cell_volt();
	volt_calc(0,NULL);
	for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
		printf("seg: %d vmin: %d\r\n",curr_ic,a_d->seg[curr_ic].v_min);
	}
	coll_unbalanced_cells();
	while(a_d->seg[0].tap > 0||a_d->seg[1].tap > 0||a_d->seg[2].tap > 0||a_d->seg[3].tap > 0||a_d->seg[4].tap > 0){
		if(stp == 0){//if balancing is not active
			//turns on balancing for all pins above threshold
			for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
				for(int i = 0; i<a_d->seg[curr_ic].tap;i++){
					a_d->s_pin = a_d->seg[curr_ic].cvnb[i];
					printf("sPIN: %d\r\n",a_d->s_pin);
					bal_cell(curr_ic);
					//cb_test();
				}
			}
				HAL_Delay(1000);
				stp = 1;
		}

		for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
			old_tap = a_d->seg[curr_ic].tap;
		}
		coll_unbalanced_cells();
		//print_cells(DATALOG_DISABLED);
		for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
			if(old_tap > a_d->seg[curr_ic].tap){
				stop_balance(0,NULL);
				printf("1 cell finished! %d left",a_d->seg[curr_ic].tap);
			}
		}
		u_sleep(1500);//sleeps so other functions can continue
		//osDelay(50);
	}
	printf("Balancing Done\r\n");
	coll_cell_volt();
}

void bal_cell(uint8_t segment){
	s_pin_read = a_d->s_pin;
    //s_pin_read = select_s_pin();
    //s_pin_read = 4;
    wakeup_sleep(TOTAL_IC);
    //LTC6813_set_discharge(s_pin_read,TOTAL_IC,a_d->BMS_IC);
    LTC6813_set_discharge_per_segment(s_pin_read,segment,a_d->BMS_IC);
    LTC6813_wrcfg(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC,a_d->BMS_IC);
    print_wrconfig();
    print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    print_rxconfig();
    print_rxconfigb();
}

void coll_unbalanced_cells(void){
	coll_cell_volt();
	//print_cells(DATALOG_DISABLED);
	for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
	  {
		a_d->seg[current_ic].tap = 0;
		for (uint8_t j = 0; j<18; j++){
			if(a_d->BMS_IC[current_ic].cells.c_codes[j] >= (a_d->seg[current_ic].v_min)+100){//ex 3.5v = 35000 so adding .05v=500 to leeway to balancing
				a_d->seg[current_ic].cvnb[a_d->seg[current_ic].tap] = j;
				a_d->seg[current_ic].tap += 1;
				printf("j: %d tap %d, cvnb:%d\r\n",j,a_d->seg[current_ic].tap,a_d->seg[current_ic].cvnb[a_d->seg[current_ic].tap]);
			}
		}
	  }
}

void temp_calc(uint8_t nargs, char **args){
    /************************************************************
      Ensure to set the GPIO bits to 1 in the CFG register group.
    *************************************************************/
	uint8_t adgid = 0b1001100;
	printf("%x",adgid);
    for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
    {
      //Communication control bits and communication data bytes. Refer to the data sheet.
		a_d->BMS_IC[current_ic].com.tx_data[0]= 0x6A; // Icom Start (6) + I2C_address D0 (A0) (Write operation to set the word address)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0x08; // Fcom master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0) + eeprom address(word address) D1 (0x00)
		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x08; // Fcom master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x6A; // Icom Start (6) + I2C_address D2 (0xA1)(Read operation)
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0x18; // Fcom master NACK(8)
    }
    wakeup_sleep(TOTAL_IC);
    LTC6813_wrcomm(TOTAL_IC,a_d->BMS_IC); // write to comm register

    wakeup_idle(TOTAL_IC);
    LTC6813_stcomm(3); // data length=3 // initiates communication between master and the I2C slave

    for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
    {
      //Communication control bits and communication data bytes. Refer to the data sheet.
		a_d->BMS_IC[current_ic].com.tx_data[0]= 0x0F; // Icom Blank (0) + data D0 (FF)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0xF9; // Fcom master NACK + Stop(9)
		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x7F; // Icom No Transmit (7) + data D1 (FF)
		a_d->BMS_IC[current_ic].com.tx_data[3]= 0xF9; // Fcom master NACK + Stop(9)
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x7F; // Icom No Transmit (7) + data D2 (FF)
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0xF9; // Fcom master NACK + Stop(9)
    }
    wakeup_idle(TOTAL_IC);
    LTC6813_wrcomm(TOTAL_IC,a_d->BMS_IC); // write to comm register

    wakeup_idle(TOTAL_IC);
    LTC6813_stcomm(1); // data length=1 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcomm(TOTAL_IC,a_d->BMS_IC); // read from comm register
    check_error(error);
    print_rxcomm(); // print received data from the comm register
}

void get_cell_data(uint8_t nargs, char **args){
	printf("\r\nTotal Cells: %d\
			\r\nTotal IC:    %d\
			\r\nVolt Min:    %.04f\
			\r\nVolt Max:    %.04f\
			\r\nVolt Avg:    %.04f\r\n",0,TOTAL_IC,a_d->seg[0].v_min*.0001,a_d->seg[0].v_max*.0001,a_d->seg[0].v_avg*.0001);
}

void fan_control(uint8_t nargs, char **args){
	int32_t dutyCycle = 0;
	if(nargs == 1){
		if(strcmp(args[1], "off") == 0){
			dutyCycle=0;
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
		}
		else if(strcmp(args[1], "half") == 0){
			dutyCycle=65535/2;
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
		}
		else if(strcmp(args[1], "max") == 0){
			dutyCycle=65535;
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
		}
		else{
			printf("Incorrect ARG 1 Input\r\n");
		}
	}
	else{
		printf("Incorrect number of ARGS should be 1");
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

void charging_mode(){//activated by GPIO Signal going high from external source(interrupt)
	a_d->mode = 0;
}

void discharge_mode(){//default mode ~when GPIO Signal is low
	a_d->mode = 1;
}

void balancing_mode(){
	a_d->mode = 2;
}

void vmon(){
	a_d->VDisp = 1;
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6813
 to the serial port.
 @return void
 ********************************************************************************/
void print_wrconfig(void)
{
    int cfg_pec;
    //Serial.println(F("Written Configuration A Register: "));
    printf("Written Configuration A Register: \r\n");
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    {
      //Serial.print(F("CFGA IC "));
      //Serial.print(current_ic+1,DEC);
      printf("CFGA IC %d",current_ic+1);
      for(int i = 0;i<6;i++)
      {
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
        printf(", %.02x",a_d->BMS_IC[current_ic].config.tx_data[i]);
      }
      //Serial.print(F(", Calculated PEC: 0x"));
      cfg_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].config.tx_data[0]);
      //serial_print_hex((uint8_t)(cfg_pec>>8));
      //Serial.print(F(", 0x"));
      //serial_print_hex((uint8_t)(cfg_pec));
      //Serial.println("\n");
      printf(", Calculated PEC: %.02x\r\n",cfg_pec);
    }
}

/*!******************************************************************************
 \brief Prints the Configuration Register B data that is going to be written to
 the LTC6813 to the serial port.
  @return void
 ********************************************************************************/
void print_wrconfigb(void)
{
    int cfg_pec;
    //Serial.println(F("Written Configuration B Register: "));
    printf("Written Configuration B Register: \r\n");
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    {
      //Serial.print(F("CFGB IC "));
      //Serial.print(current_ic+1,DEC);
      printf("CFGB IC %d",current_ic+1);
      for(int i = 0;i<6;i++)
      {
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].configb.tx_data[i]);
        printf(", %.02x",a_d->BMS_IC[current_ic].config.tx_data[i]);
      }
      //Serial.print(F(", Calculated PEC: 0x"));
      cfg_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].configb.tx_data[0]);
      //serial_print_hex((uint8_t)(cfg_pec>>8));
      //Serial.print(F(", 0x"));
      //serial_print_hex((uint8_t)(cfg_pec));
      //Serial.println("\n");
      printf(", Calculated PEC: %.02x\r\n",cfg_pec);
    }
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6813 to the serial port.
 @return void
 *******************************************************************/
void print_rxconfig(void)
{
  //Serial.println(F("Received Configuration A Register: "));
  printf("Received Configuration A Register: \r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F("CFGA IC "));
    //Serial.print(current_ic+1,DEC);
    printf("CFGA IC %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
      printf(", %.02x",a_d->BMS_IC[current_ic].config.rx_data[i]);
    }
    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].config.rx_data[6],a_d->BMS_IC[current_ic].config.rx_data[7]);
  }
}

/*!*****************************************************************
 \brief Prints the Configuration Register B that was read back from
 the LTC6813 to the serial port.
  @return void
 *******************************************************************/
void print_rxconfigb(void)
{
  //Serial.println(F("Received Configuration B Register: "));
  printf("Received Configuration B Register: \r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F("CFGB IC "));
    //Serial.print(current_ic+1,DEC);
    printf("CFGB IC %d",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].configb.rx_data[i]);
      printf(", %.02x",a_d->BMS_IC[current_ic].configb.rx_data[i]);
    }
    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].configb.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].configb.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].configb.rx_data[6],a_d->BMS_IC[current_ic].configb.rx_data[7]);
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
      for (int i=0; i<a_d->BMS_IC[0].ic_reg.cell_channels; i++)
      {
        //Serial.print(" C");
        //Serial.print(i+1,DEC);
        //Serial.print(":");
        //Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        //Serial.print(",");
        printf(" C%d:%.4f,",i+1,a_d->BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
      //Serial.println();
      printf("\r\n");
    }
    else
    {
      //Serial.print(" Cells, ");
      printf(" Cells, ");
      for (int i=0; i<a_d->BMS_IC[0].ic_reg.cell_channels; i++)
      {
        //Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
        //Serial.print(",");
        printf("%f,",a_d->BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
    }
  }
  //Serial.println("\n");
  //printf("\r\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      //Serial.print(" IC ");
      //Serial.print(current_ic+1,DEC);
      printf(" IC %d",current_ic+1);
      for (int i=0; i < 5; i++)
      {
        //Serial.print(F(" GPIO-"));
        //Serial.print(i+1,DEC);
        //Serial.print(":");
        //Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        //Serial.print(",");
        printf(" GPIO-%d:%f,",i+1,a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }

      for (int i=6; i < 10; i++)
      {
        //Serial.print(F(" GPIO-"));
        //Serial.print(i,DEC);
        //Serial.print(":");
        //Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        printf(" GPIO-%d:%f,",i+1,a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }

      //Serial.print(F(" Vref2"));
      //Serial.print(":");
      //Serial.print(BMS_IC[current_ic].aux.a_codes[5]*0.0001,4);
      //Serial.println();

      //Serial.print(" OV/UV Flags : 0x");
      //Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[11],HEX);
      //Serial.println();
      printf(" Vref2:%f\r\n OV/UV Flags : %x\r\n",a_d->BMS_IC[current_ic].aux.a_codes[5]*0.0001,(uint8_t)a_d->BMS_IC[current_ic].aux.a_codes[11]);
    }
    else
    {
      //Serial.print(" AUX, ");
      printf(" AUX, ");

      for (int i=0; i < 12; i++)
      {
        //Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        //Serial.print(",");
        printf("%f,",(uint8_t)a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }
    }
  }
 //Serial.println("\n");
 printf("\r\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
  @return void
 *****************************************************************************/
void print_stat(void)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    double itmp;

    itmp = (double)((a_d->BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0076)) - 276);   //Internal Die Temperature(°C) = ITMP • (100 µV / 7.6mV)°C - 276°C
    /*Serial.print(F(" IC "));
    Serial.print(current_ic+1,DEC);
    Serial.print(F(" SOC:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30,4);
    Serial.print(F(","));
    Serial.print(F(" Itemp:"));
    itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0076)) - 276);   //Internal Die Temperature(°C) = ITMP • (100 µV / 7.6mV)°C - 276°C
    Serial.print(itmp,4);
    Serial.print(F(","));
    Serial.print(F(" VregA:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[2]*0.0001,4);
    Serial.print(F(","));
    Serial.print(F(" VregD:"));
    Serial.print(BMS_IC[current_ic].stat.stat_codes[3]*0.0001,4);
    Serial.println();*/
    printf(" IC %d SOC:%f, Itemp:%f, VregA:%f, VregD:%f\r\n",current_ic+1,a_d->BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30,itmp,a_d->BMS_IC[current_ic].stat.stat_codes[2]*0.0001,a_d->BMS_IC[current_ic].stat.stat_codes[3]*0.0001);
    //Serial.print(F(" OV/UV Flags:"));
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].stat.flags[1]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].stat.flags[2]);
     //Serial.print(F("\tMux fail flag:"));
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
     //Serial.print(F("\tTHSD:"));
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
    //Serial.println();
    printf(" OV/UV Flags:, %.02x, %.02x, %.02x\tMux fail flag: %.02x\tTHSD:, %.02x\r\n",a_d->BMS_IC[current_ic].stat.flags[0],a_d->BMS_IC[current_ic].stat.flags[1],a_d->BMS_IC[current_ic].stat.flags[2],a_d->BMS_IC[current_ic].stat.mux_fail[0],a_d->BMS_IC[current_ic].stat.thsd[0]);
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes (GPIO 1 & 2)
  @return void
 *****************************************************************************/
void print_aux1(uint8_t datalog_en)
{

  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      //Serial.print(" IC ");
      //Serial.print(current_ic+1,DEC);
      printf(" IC %d",current_ic+1);
      for (int i=0; i < 2; i++)
      {
        //Serial.print(F(" GPIO-"));
        //Serial.print(i+1,DEC);
        //Serial.print(":");
        //Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        //Serial.print(",");
        printf(" GPIO-%d:%f,",i+1,a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }
    }
    else
    {
      //Serial.print("AUX, ");
      printf("AUX, ");

      for (int i=0; i < 12; i++)
      {
        //Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
        //Serial.print(",");
        printf("%f,",a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }
    }
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
 *****************************************************************************/
void print_sumofcells(void)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC "));
    //Serial.print(current_ic+1,DEC);
    //Serial.print(F(" SOC:"));
    //Serial.print(BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30,4);
    //Serial.print(F(","));
    printf(" IC %d SOC:%f,",current_ic+1,a_d->BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30);
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************
  \brief Function to check the MUX fail bit in the Status Register
   @return void
*******************************************************************/
void check_mux_fail(void)
{
  int8_t error = 0;
  for (int ic = 0; ic<TOTAL_IC; ic++)
    {
      //Serial.print(" IC ");
      //Serial.println(ic+1,DEC);
      printf(" IC %d,",ic+1);
      if (a_d->BMS_IC[ic].stat.mux_fail[0] != 0) error++;

      if (error==0) printf("MUX Test: PASS \r\n");//Serial.println(F("Mux Test: PASS \n"));
      else printf("Mux Test: FAIL \r\n");//Serial.println(F("Mux Test: FAIL \n"));
    }
}

/*!************************************************************
  \brief Prints Errors Detected during self test
   @return void
*************************************************************/
void print_selftest_errors(uint8_t adc_reg ,int8_t error)
{
  if(adc_reg==1)
  {
    //Serial.println("Cell ");
    printf("Cell \r\n");
    }
  else if(adc_reg==2)
  {
    //Serial.println("Aux ");
    printf("Aux \r\n");
    }
  else if(adc_reg==3)
  {
    //Serial.println("Stat ");
    printf("Stat \r\n");
    }
  //Serial.print(error, DEC);
  //Serial.println(F(" : errors detected in Digital Filter and Memory \n"));
  printf("%d : errors detected in Digital Filter and Memory\r\n",error);
}

/*!************************************************************
  \brief Prints the output of  the ADC overlap test
   @return void
*************************************************************/
void print_overlap_results(int8_t error)
{
  if (error==0) printf("Overlap Test: PASS \r\n");//Serial.println(F("Overlap Test: PASS \n"));
  else printf("Overlap Test: FAIL \r\n");//Serial.println(F("Overlap Test: FAIL \n"));
}

/*!************************************************************
  \brief Prints Errors Detected during Digital Redundancy test
   @return void
*************************************************************/
void print_digital_redundancy_errors(uint8_t adc_reg ,int8_t error)
{
  if(adc_reg==2)
  {
    //Serial.println("Aux ");
    printf("Aux \r\n");
    }
  else if(adc_reg==3)
  {
    //Serial.println("Stat ");
    printf("Stat \r\n");
    }

  //Serial.print(error, DEC);
  //Serial.println(F(" : errors detected in Measurement \n"));
  printf("%d : errors detected in Measurement\r\n",error);
}

/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open_wires(void)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (a_d->BMS_IC[current_ic].system_open_wire == 65535)
    {
      //Serial.print("No Opens Detected on IC ");
      //Serial.print(current_ic+1, DEC);
      //Serial.println();
      printf("No Opens Detected on IC %d\r\n",current_ic+1);
    }
    else
    {
      //Serial.print(F("There is an open wire on IC "));
      //Serial.print(current_ic + 1,DEC);
      //Serial.print(F(" Channel: "));
      //Serial.println(BMS_IC[current_ic].system_open_wire);
      printf("There is an open wire on IC %d Channel: %ld\r\n",current_ic+1,a_d->BMS_IC[current_ic].system_open_wire);
    }
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************************
   \brief Function to print the number of PEC Errors
   @return void
 *****************************************************************************/
void print_pec_error_count(void)
{
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
      //Serial.println("");
      //Serial.print(BMS_IC[current_ic].crc_count.pec_count,DEC);
      //Serial.print(F(" : PEC Errors Detected on IC"));
      //Serial.println(current_ic+1,DEC);
      printf("\r\n%d : PEC Errors Detected on IC%d",a_d->BMS_IC[current_ic].crc_count.pec_count,current_ic+1);
  }
  //Serial.println("\n");
  printf("\r\n");
}

/*!****************************************************************************
  \brief prints data which is written on PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrpwm(void)
{
  int pwm_pec;

  //Serial.println(F("Written PWM Configuration: "));
  printf("Written PWM Configuration: \r\n");
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F("IC "));
    //Serial.print(current_ic+1,DEC);
    printf("IC %d",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
     //serial_print_hex(BMS_IC[current_ic].pwm.tx_data[i]);
     printf(", %.02x",a_d->BMS_IC[current_ic].pwm.tx_data[i]);
    }
    //Serial.print(F(", Calculated PEC: 0x"));
    pwm_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].pwm.tx_data[0]);
    //serial_print_hex((uint8_t)(pwm_pec>>8));
    //Serial.print(F(", 0x"));
    //serial_print_hex((uint8_t)(pwm_pec));
    //Serial.println("\n");
    printf(", Calculated PEC: %.02x, %.02x\r\n",(uint8_t)(pwm_pec>>8),(uint8_t)(pwm_pec));
  }
}

/*!****************************************************************************
  \brief Prints received data from PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxpwm(void)
{
  //Serial.println(F("Received pwm Configuration:"));
  printf("Received pwm Configuration:\r\n");
  for (uint8_t current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F("IC "));
    //Serial.print(current_ic+1,DEC);
    printf("IC %d",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
     //serial_print_hex(BMS_IC[current_ic].pwm.rx_data[i]);
     printf(", %.02x",a_d->BMS_IC[current_ic].pwm.rx_data[i]);
    }
    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].pwm.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].pwm.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].pwm.rx_data[6],a_d->BMS_IC[current_ic].pwm.rx_data[7]);
  }
}

/*!****************************************************************************
  \brief prints data which is written on S Control register
  @return void
 *****************************************************************************/
void print_wrsctrl(void)
{
    int sctrl_pec;

  //Serial.println(F("Written Data in Sctrl register: "));
  printf("Written Data in Sctrl register: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC: "));
    //Serial.print(current_ic+1,DEC);
    //Serial.print(F(" Sctrl register group:"));
    printf(" IC: %d Sctrl register group:",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
      //Serial.print(F(", 0x"));
      //serial_print_hex(BMS_IC[current_ic].sctrl.tx_data[i]);
      printf(", %.02x",a_d->BMS_IC[current_ic].sctrl.tx_data[i]);
    }

    //Serial.print(F(", Calculated PEC: 0x"));
    sctrl_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].sctrl.tx_data[0]);
    //serial_print_hex((uint8_t)(sctrl_pec>>8));
    //Serial.print(F(", 0x"));
    //serial_print_hex((uint8_t)(sctrl_pec));
    //Serial.println("\n");
    printf(", Calculated PEC: %.02x, %.02x\r\n",(uint8_t)(sctrl_pec>>8),(uint8_t)(sctrl_pec));
  }
}

/*!****************************************************************************
  \brief prints data which is read back from S Control register
  @return void
 *****************************************************************************/
void print_rxsctrl(void)
{
   //Serial.println(F("Received Data:"));
   printf("Received Data:\r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" IC "));
    //Serial.print(current_ic+1,DEC);
    printf(" IC %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[i]);
    printf(", %.02x",a_d->BMS_IC[current_ic].sctrl.rx_data[i]);
    }

    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].sctrl.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].sctrl.rx_data[6],a_d->BMS_IC[current_ic].sctrl.rx_data[7]);
  }
}

/*!****************************************************************************
  \brief Prints data which is written on PWM/S control register group B onto
  the serial port
   @return void
 *****************************************************************************/
void print_wrpsb(uint8_t type)
{
  int psb_pec=0;

  //Serial.println(F(" PWM/S control register group B: "));
  printf(" PWM/S control register group B: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
      if(type == 1)
      {
        //Serial.print(F(" IC: "));
        //Serial.println(current_ic+1,DEC);
        //Serial.print(F(" 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[0]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[1]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.tx_data[2]);
        printf(" IC %d, %.02x, %.02x, %.02x",current_ic+1,a_d->BMS_IC[current_ic].pwmb.tx_data[0],a_d->BMS_IC[current_ic].pwmb.tx_data[1],a_d->BMS_IC[current_ic].pwmb.tx_data[2]);

        //Serial.print(F(", Calculated PEC: 0x"));
        psb_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].pwmb.tx_data[0]);
        //serial_print_hex((uint8_t)(psb_pec>>8));
        //Serial.print(F(", 0x"));
        //serial_print_hex((uint8_t)(psb_pec));
        //Serial.println("\n");
        printf(", Calculated PEC: %.02x, %.02x\r\n",(uint8_t)(psb_pec>>8),(uint8_t)(psb_pec));
      }
      else if(type == 2)
      {
        //Serial.print(F(" IC: "));
        //Serial.println(current_ic+1,DEC);
        printf(" IC %d\r\n",current_ic+1);
        //Serial.print(F(" 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[3]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[4]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.tx_data[5]);

        //Serial.print(F(", Calculated PEC: 0x"));
        psb_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].sctrlb.tx_data[0]);
        //serial_print_hex((uint8_t)(psb_pec>>8));
        //Serial.print(F(", 0x"));
        //serial_print_hex((uint8_t)(psb_pec));
        //Serial.println("\n");
        printf(" %.02x, %.02x, %.02x, Calculated PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].sctrlb.tx_data[3],a_d->BMS_IC[current_ic].sctrlb.tx_data[4],a_d->BMS_IC[current_ic].sctrlb.tx_data[5],(uint8_t)(psb_pec>>8),(uint8_t)(psb_pec));
      }
  }
}


/*!****************************************************************************
  \brief Prints received data from PWM/S control register group B
   onto the serial port
   @return void
 *****************************************************************************/
void print_rxpsb(uint8_t type)
{
  //Serial.println(F(" PWM/S control register group B:"));
  printf(" PWM/S control register group B:\r\n");
  if(type == 1)
  {
      for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
      {
        //Serial.print(F(" IC: "));
        //Serial.println(current_ic+1,DEC);
        printf(" IC: %d \r\n",current_ic+1);
        //Serial.print(F(" 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[0]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[1]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[2]);

        //Serial.print(F(", Received PEC: 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[6]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].pwmb.rx_data[7]);
       //Serial.println("\n");
       printf(" %.02x, %.02x, %.02x, Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].pwmb.rx_data[0],a_d->BMS_IC[current_ic].pwmb.rx_data[1],a_d->BMS_IC[current_ic].pwmb.rx_data[2],a_d->BMS_IC[current_ic].pwmb.rx_data[6],a_d->BMS_IC[current_ic].pwmb.rx_data[7]);

      }
  }
   else if(type == 2)
  {
      for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
      {
        //Serial.print(F(" IC: "));
        //Serial.println(current_ic+1,DEC);
        printf(" IC: %d\r\n",current_ic+1);
        //Serial.print(F(" 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[3]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[4]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[5]);

        //Serial.print(F(", Received PEC: 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[6]);
        //Serial.print(F(", 0x"));
        //serial_print_hex(BMS_IC[current_ic].sctrlb.rx_data[7]);
        //Serial.println("\n");
        printf(" %.02x, %.02x, %.02x, Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].sctrlb.rx_data[3],a_d->BMS_IC[current_ic].sctrlb.rx_data[4],a_d->BMS_IC[current_ic].sctrlb.rx_data[5],a_d->BMS_IC[current_ic].sctrlb.rx_data[6],a_d->BMS_IC[current_ic].sctrlb.rx_data[7]);
      }
  }
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
      printf(", %x",a_d->BMS_IC[current_ic].com.tx_data[i]);
    }
    //Serial.print(F(", Calculated PEC: 0x"));
    printf(", Calculated PEC: ");
    comm_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].com.tx_data[0]);
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
      printf(", %x",a_d->BMS_IC[current_ic].com.rx_data[i]);
    }
    //Serial.print(F(", Received PEC: 0x"));
    //serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
    //Serial.print(F(", 0x"));
    //serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
    //Serial.println("\n");
    printf(", Received PEC: %x, %x\r\n",a_d->BMS_IC[current_ic].com.rx_data[6],a_d->BMS_IC[current_ic].com.rx_data[7]);
  }
}

/*!********************************************************************
  \brief Function to check the Mute bit in the Configuration Register
   @return void
**********************************************************************/
void check_mute_bit(void)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    //Serial.print(F(" Mute bit in Configuration Register B: 0x"));
    //serial_print_hex((BMS_IC[current_ic].configb.rx_data[1])&(0x80));
    //Serial.println("\n");
    printf(" Mute bit in Configuration Register B: %.02x\r\n",(a_d->BMS_IC[current_ic].configb.rx_data[1])&(0x80));
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


void init_app_data_bms(app_data *app_data_init)
{
	a_d = app_data_init;
}

