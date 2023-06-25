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

void chg_string(uint8_t nargs, char **args){
	if(nargs == 1){
		if(strcmp(args[1], "0") == 0){
			a_d->ltcstring = 0;
			//printf("\r\n%d\r\n",a_d->ltcstring);
		}
		else if(strcmp(args[1], "1") == 0){
			a_d->ltcstring = 1;
			//printf("\r\n%d\r\n",a_d->ltcstring);
		}
		else{
			;
		}
	}
	else{
		;
	}
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
				if(0<atoi(args[2])&&atoi(args[2])<150){
					a_d->max_curr = atoi(args[2]);
				}
				//printf("value is now %f",*a_d->max_curr);
			}
			else if(strcmp(args[1], "MACV") == 0){
				if(0<atoi(args[2])&&atoi(args[2])<6.6){
					a_d->max_cell_volt = atoi(args[2]);
				}
				//printf("value is now %f",*a_d->max_cell_volt);

			}
			else if(strcmp(args[1], "MICV") == 0){
				if(0<atoi(args[2])&&atoi(args[2])<6.6){
					a_d->min_cell_volt = atof(args[2]);
				}
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
			balancing_mode();
		}
		else if(strcmp(args[1], "no") == 0){
			a_d->mode = 127;
		}
		else{
			printf("Incorrect Arg\r\n");
		}
	}
	else if(nargs ==0){
		printf("Mode: %d\r\n",a_d->mode);
	}
	else{
		printf("too few/many arguments\r\n");
	}
}

void volt_calc(uint8_t nargs, char **args){//collects voltages across all ICs calculate minimum, maximum and avg voltage per segment
	uint16_t volt_min=65535,volt_max=0,total_cells=0;
	uint32_t volt_total=0;
	for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
	  {
		volt_min=65535;volt_max=0;total_cells=0;
	    for (int i=0; i<a_d->BMS_IC[current_ic].ic_reg.cell_channels; i++){
	    	//printf("%d:%.4f\r\n",BMS_IC[current_ic].cells.c_codes[i],BMS_IC[current_ic].cells.c_codes[i]*0.0001);

	    	if(a_d->BMS_IC[current_ic].cells.c_codes[i]!=65535&&a_d->BMS_IC[current_ic].cells.c_codes[i]>=10000){
	    		if(volt_min>a_d->BMS_IC[current_ic].cells.c_codes[i]){
					volt_min = a_d->BMS_IC[current_ic].cells.c_codes[i];
				}

	    		if(volt_max<a_d->BMS_IC[current_ic].cells.c_codes[i]){
					volt_max=a_d->BMS_IC[current_ic].cells.c_codes[i];
				}

				volt_total += a_d->BMS_IC[current_ic].cells.c_codes[i];
				total_cells++;
	    	}
	    }
	    a_d->seg[current_ic].v_max = volt_max;
		a_d->seg[current_ic].v_min = volt_min;
		a_d->seg[current_ic].v_tot = volt_total;
		a_d->seg[current_ic].v_avg = volt_total/total_cells;
	  }
    //printf("total cells: %d\r\n",total_cells);
    //printf("volt total %d, volt_avg %d\r\n",volt_total,volt_avg);
    //printf("vmax: %d, vmin %d, vavg, %d\r\n",volt_max,volt_min,volt_avg);
}

void coll_cell_volt(void){//uint8_t nargs, char **args){
	wakeup_sleep(TOTAL_IC);
	LTC6813_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
	conv_time = LTC6813_pollAdc();
	//print_conv_time(conv_time);  //gotta fix this whole part

    wakeup_sleep(TOTAL_IC);
    do{
    	error = LTC6813_rdcv(SEL_ALL_REG,TOTAL_IC,a_d->BMS_IC); // Set to read back all cell voltage registers
    	check_error(error);
    } while(error == -1);
	volt_calc(0,NULL);
    if(a_d->VDisp == 1){
    	if(__HAL_TIM_GET_COUNTER(a_d->htim12) > 10000){
        	print_cells(DATALOG_DISABLED);
        	__HAL_TIM_SET_COUNTER(a_d->htim12,0);
        	printf("\r\n\r\n");
        	//printf("current voltage: %lu\r\n",HAL_ADC_GetValue(a_d->hadc));
    	}
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
    //print_wrconfig();
    //print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    do{
		error = LTC6813_rdcfg(TOTAL_IC,a_d->BMS_IC);
		check_error(error);
	}while (error == -1);

    do{
		error = LTC6813_rdcfgb(TOTAL_IC,a_d->BMS_IC);
		check_error(error);
    }while (error == -1);
    //print_rxconfig();
    //print_rxconfigb();
    printf("Discharging Cell: %d\r\n",s_pin_read);
}

void stop_balance(uint8_t nargs, char **args){
    wakeup_sleep(TOTAL_IC);
    LTC6813_clear_discharge(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfg(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC,a_d->BMS_IC);
    //print_wrconfig();
    //print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    do{
		error = LTC6813_rdcfg(TOTAL_IC,a_d->BMS_IC);
		check_error(error);
    }while (error == -1);
    do{
		error = LTC6813_rdcfgb(TOTAL_IC,a_d->BMS_IC);
		check_error(error);
    }while (error == -1);
    //print_rxconfig();
    //print_rxconfigb();
    printf("balance stopped\r\n");
}

/*
 * Collects all voltages and balances every cell in segment to lowest voltage
 */
void bal_all(uint8_t nargs, char **args){
	for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
		a_d->seg[curr_ic].old_mask = 65535;//16 cells active
	}
	//was balancing stopped? 0 yes 1 no
	uint8_t stp = 0;
	printf("Starting Balancing\r\n");
	//coll_cell_volt();
	//volt_calc(0,NULL);
	coll_unbalanced_cells();
	for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
		printf("seg: %d vmin: %d\r\n",curr_ic,a_d->seg[curr_ic].v_min);
	}

	while(a_d->seg[0].volt_mask>0||a_d->seg[1].volt_mask>0||a_d->seg[2].volt_mask>0||a_d->seg[3].volt_mask>0||a_d->seg[4].volt_mask>0){
		if(stp == 0){//if balancing is not active
			//turns on balancing for all pins above threshold
			for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
				printf("Discharging: ");
				for(int i = 0; i<18;i++){
					if((a_d->seg[curr_ic].volt_mask>>i)&1){
						printf("%d,",i+1);
						a_d->s_pin = i+1;//maybe remove +1
						LTC6813_set_discharge_per_segment(a_d->s_pin,curr_ic,a_d->BMS_IC);
					}
				}
			}
				//writes config for each cell to be balanced
				printf("\r\nBalancing's a go\r\n");
				bal_cell(0);
				osDelay(1000);
				stp = 1;
		}

		for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
			a_d->seg[curr_ic].old_mask = a_d->seg[curr_ic].volt_mask;
		}
		coll_unbalanced_cells();
		//print_cells(DATALOG_DISABLED);
		for(int curr_ic = 0; curr_ic<TOTAL_IC; curr_ic++){
			if(a_d->seg[curr_ic].old_mask != a_d->seg[curr_ic].volt_mask){
				stop_balance(0,NULL);//maybe change
				stp = 0;
				printf("Seg%d: 1 cell finished! %d mask",curr_ic,a_d->seg[curr_ic].volt_mask);
			}
		}
		u_sleep(1500);//sleeps so other functions can continue
		//osDelay(50);
	}
	printf("\r\nBalancing Done\r\n");
	a_d->mode = 127;
	coll_cell_volt();
}

void bal_cell(uint8_t segment){
	//s_pin_read = a_d->s_pin;
    //s_pin_read = select_s_pin();
    //s_pin_read = 4;
    wakeup_sleep(TOTAL_IC);
    //LTC6813_set_discharge(s_pin_read,TOTAL_IC,a_d->BMS_IC);
    //LTC6813_set_discharge_per_segment(s_pin_read,segment,a_d->BMS_IC);
    LTC6813_wrcfg(TOTAL_IC,a_d->BMS_IC);
    LTC6813_wrcfgb(TOTAL_IC,a_d->BMS_IC);
    //print_wrconfig();
    //print_wrconfigb();
    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcfg(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    error = LTC6813_rdcfgb(TOTAL_IC,a_d->BMS_IC);
    check_error(error);
    //print_rxconfig();
    //print_rxconfigb();
}

void coll_unbalanced_cells(void){
	coll_cell_volt();
	for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++){
		a_d->seg[current_ic].volt_mask = 0;
		for (uint8_t j = 0; j<18; j++){//create loop if voltage equals 65535 rerun coll_cell_volt
			if(a_d->BMS_IC[current_ic].cells.c_codes[j] >= (a_d->seg[current_ic].v_min)+100){//ex 3.5v = 35000 so adding .05v=500 to leeway to balancing
				a_d->seg[current_ic].volt_mask |= 1<<j;
			}
		}
		if(a_d->seg[current_ic].volt_mask > a_d->seg[current_ic].old_mask){//or set the new mask to old mask... tbd
			//a_d->seg[current_ic].repeat_coll = 1;
			printf("new mask greater than old mask\r\n");
			//a_d->seg[current_ic].volt_mask = a_d->seg[current_ic].old_mask;
		}
		else{
			//a_d->seg[current_ic].repeat_coll = 0;
		}
	}
	if(a_d->seg[0].repeat_coll||a_d->seg[1].repeat_coll||a_d->seg[2].repeat_coll||a_d->seg[3].repeat_coll||a_d->seg[4].repeat_coll){
		coll_unbalanced_cells();
		printf("need to repeat\r\n");
	}
}

void temp_test(void){
	temp_calc(7);
	for(int i = 0; i<20; i++){
		wakeup_sleep(TOTAL_IC);
		LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
		conv_time = LTC6813_pollAdc();
		print_conv_time(conv_time);
		wakeup_sleep(TOTAL_IC);
		error = LTC6813_rdaux(SEL_ALL_REG,TOTAL_IC,a_d->BMS_IC); // Set to read back all aux registers
		check_error(error);
		print_aux(DATALOG_DISABLED);
		osDelay(1000);
	}
	/*
	for(int i=0;i<8;i++){
		temp_calc(i);
		printf("\r\n%d\r\n",i);
		//for testing read T Cell 14... on mux 00
		for(int i = 0; i<4; i++){
			wakeup_sleep(TOTAL_IC);
			LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
			conv_time = LTC6813_pollAdc();
			print_conv_time(conv_time);
			wakeup_sleep(TOTAL_IC);
			error = LTC6813_rdaux(SEL_ALL_REG,TOTAL_IC,a_d->BMS_IC); // Set to read back all aux registers
			check_error(error);
			print_aux(DATALOG_DISABLED);
			osDelay(1000);
		}
	}*/
}

void temp_calc(uint8_t channel){
    /************************************************************
      Ensure to set the GPIO bits to 1 in the CFG register group.
    *************************************************************/
	//5 MSB is idetnifier for mux 2 lsb is identifier of specific mux last bit signifies write or read
	//uint8_t adgid = 0b10011000;
	//printf("%x",adgid);
    for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
    {
      //Communication control bits and communication data bytes. Refer to the data sheet.
		/*a_d->BMS_IC[current_ic].com.tx_data[0]= 0x6A; // Icom Start (6) + I2C_address D0 (A0) (Write operation to set the word address)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0x08; // Fcom master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0) + eeprom address(word address) D1 (0x00)
		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x08; // Fcom master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x6A; // Icom Start (6) + I2C_address D2 (0xA1)(Read operation)
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0x18; // Fcom master NACK(8)

	     wakeup_sleep(TOTAL_IC);
	      LTC6813_wrcomm(TOTAL_IC,a_d->BMS_IC); // write to comm register

	      wakeup_idle(TOTAL_IC);
	      LTC6813_stcomm(3);
	      //old*/
        /*a_d->BMS_IC[current_ic].com.tx_data[0]= 0x69;//Icom Start(6) + 1001(0x9)
        a_d->BMS_IC[current_ic].com.tx_data[1]= 0x88;//1000 + master NACK(8)
        a_d->BMS_IC[current_ic].com.tx_data[2]= 0x04;//blank + 0100(only 7 on)
        a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09;//0000(4-1 off mux channel) + master NACK(8)+stop
        a_d->BMS_IC[current_ic].com.tx_data[4]= 0x7F;//0111 no transmit
        a_d->BMS_IC[current_ic].com.tx_data[5]= 0xF9;//1001 master nack + STOP
        */
        //new
    	//7,6,5,4,3,2,1,0
    	if(channel > 7){
    		printf("Channel size not in params\r\n");
    	}
    	else if(channel > 3){
    		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x0|(1<<(channel-4));//blank + 0100(only 7 on)
    		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09;//0000(4-1 off mux channel) + master NACK(8)+stop
    	}
    	else{
    		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x00;//blank + 0100(only 7 on)
    		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09|(1<<(channel+4));//0000(4-1 off mux channel) + master NACK(8)+stop
    	}
		a_d->BMS_IC[current_ic].com.tx_data[0]= 0x69;//Icom Start(6) + 1001(0x9)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0x88;//1000 + master NACK(8)
		//a_d->BMS_IC[current_ic].com.tx_data[2]= 0x04;//blank + 0100(only 7 on)
		//a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09;//0000(4-1 off mux channel) + master NACK(8)+stop
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x7F;//0111 no transmit
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0xF9;//1001 master nack + STOP


        /*a_d->BMS_IC[current_ic].com.tx_data[0]= 0b01101001;//Icom Start(6) + 1001(0x9)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0b10000000;//1000 + master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[2]= 0b00000100;//blank + 0100(only 7 on)
		a_d->BMS_IC[current_ic].com.tx_data[3]= 0b00001001;//0000(4-1 off mux channel) + master NACK(8)+stop
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x7F;//0111 no transmit
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0xF9;*/
		/*a_d->BMS_IC[current_ic].com.tx_data[0]= 0x6A; // Icom Start(6) + I2C_address D0 (0xA0)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0x08; // Fcom master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0) + eeprom address D1 (0x00)
		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x08; // Fcom master NACK(8)
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x01; // Icom Blank (0) + data D2 (0x13)
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0x39; // Fcom master NACK + Stop(9)*/
    }
    wakeup_sleep(TOTAL_IC);
    LTC6813_wrcomm(TOTAL_IC,a_d->BMS_IC); // write to comm registter

    wakeup_idle(TOTAL_IC);
    LTC6813_stcomm(3); // data length=3 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcomm(TOTAL_IC,a_d->BMS_IC); // read from comm register
    check_error(error);
    print_wrcomm();
    print_rxcomm(); // print received data from the comm register
    /*
    //2nd mux
    for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++)
    {
    	if(channel > 7){
    		printf("Channel size not in params\r\n");
    	}
    	else if(channel > 3){
    		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x0|(1<<(channel-4));//blank + 0100(only 7 on)
    		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09;//0000(4-1 off mux channel) + master NACK(8)+stop
    	}
    	else{
    		a_d->BMS_IC[current_ic].com.tx_data[2]= 0x00;//blank + 0100(only 7 on)
    		a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09|(1<<(channel+4));//0000(4-1 off mux channel) + master NACK(8)+stop
    	}
		a_d->BMS_IC[current_ic].com.tx_data[0]= 0x69;//Icom Start(6) + 1001(0x9)
		a_d->BMS_IC[current_ic].com.tx_data[1]= 0xA8;//1000 + master NACK(8)
		//a_d->BMS_IC[current_ic].com.tx_data[2]= 0x04;//blank + 0100(only 7 on)
		//a_d->BMS_IC[current_ic].com.tx_data[3]= 0x09;//0000(4-1 off mux channel) + master NACK(8)+stop
		a_d->BMS_IC[current_ic].com.tx_data[4]= 0x7F;//0111 no transmit
		a_d->BMS_IC[current_ic].com.tx_data[5]= 0xF9;//1001 master nack + STOP
    }
    wakeup_sleep(TOTAL_IC);
    LTC6813_wrcomm(TOTAL_IC,a_d->BMS_IC); // write to comm registter

    wakeup_idle(TOTAL_IC);
    LTC6813_stcomm(2); // data length=3 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6813_rdcomm(TOTAL_IC,a_d->BMS_IC); // read from comm register
    check_error(error);
    print_wrcomm();
    print_rxcomm(); // print received data from the comm register*/
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
		}
		else if(strcmp(args[1], "half") == 0){
			dutyCycle=65535/2;
		}
		else if(strcmp(args[1], "max") == 0){
			dutyCycle=65535;
		}
		else{
			printf("Incorrect ARG 1 Input\r\n");
		}
        TIM1->CCR1 = dutyCycle;TIM1->CCR2 = dutyCycle;TIM1->CCR3 = dutyCycle;TIM1->CCR4 = dutyCycle;
        TIM3->CCR1 = dutyCycle;TIM3->CCR2 = dutyCycle;TIM3->CCR3 = dutyCycle;TIM3->CCR4 = dutyCycle;
		TIM4->CCR1 = dutyCycle;TIM4->CCR2 = dutyCycle;TIM4->CCR3 = dutyCycle;TIM4->CCR4 = dutyCycle;
	}
	else{
		printf("Incorrect number of ARGS should be 1");
	}
	//for later for temp based fan editing
	while(0){
		int temp = 2;
		//temp range 0->60
		//x/100=x/60
				dutyCycle = (temp/60)*65535;
	}
}

void to_ecu(){
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

void shutdown(){
	a_d->shutdown = 1;
	a_d->mode = 127;
	stop_balance(0,NULL);
	printf("Ready to shutdown\r\n");
}

void check_mute_bit(void)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    printf(" Mute bit in Configuration Register B: %.02x\r\n",(a_d->BMS_IC[current_ic].configb.rx_data[1])&(0x80));
  }
}

void print_conv_time(uint32_t conv_time)
{
  uint16_t m_factor=1000;  // to print in ms

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

