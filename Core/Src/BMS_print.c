/*
 * BMS_print.c
 *
 *  Created on: Jun 10, 2023
 *      Author: kauff
 */

#include "BMS_print.h"

static app_data *a_d;

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6813
 to the serial port.
 @return void
 ********************************************************************************/
void print_wrconfig(void)
{
    int cfg_pec;
    printf("Written Configuration A Register: \r\n");
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    {
      printf("CFGA IC %d",current_ic+1);
      for(int i = 0;i<6;i++)
      {
        printf(", %.02x",a_d->BMS_IC[current_ic].config.tx_data[i]);
      }
      cfg_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].config.tx_data[0]);
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
    printf("Written Configuration B Register: \r\n");
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    {
      printf("CFGB IC %d",current_ic+1);
      for(int i = 0;i<6;i++)
      {
        printf(", %.02x",a_d->BMS_IC[current_ic].config.tx_data[i]);
      }
      cfg_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].configb.tx_data[0]);
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
  printf("Received Configuration A Register: \r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    printf("CFGA IC %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      printf(", %.02x",a_d->BMS_IC[current_ic].config.rx_data[i]);
    }
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
  printf("Received Configuration B Register: \r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    printf("CFGB IC %d",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
      printf(", %.02x",a_d->BMS_IC[current_ic].configb.rx_data[i]);
    }
    printf(", Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].configb.rx_data[6],a_d->BMS_IC[current_ic].configb.rx_data[7]);
  }
}

void print_cells(uint8_t datalog_en)
{
  for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++)
  {
    if (datalog_en == 0)
    {
      printf("\r\n\t-------------------------------------------------------");
      printf("\r\n\t| vmin: %f || vmax: %f || vtot: %f |",a_d->seg[current_ic].v_min*0.0001,a_d->seg[current_ic].v_max*0.0001,a_d->seg[current_ic].v_tot*0.0001);
      printf("\r\n\t-------------------------------------------------------\r\n");
      printf("\t--------\r\n");
      printf("\t| IC %d |",current_ic+1);
      printf("\r\n\t------------------------------------------------------------------------------------------------------------------------------\r\n\t");
      for (int i=0; i<a_d->BMS_IC[0].ic_reg.cell_channels/2; i++)
      {
        printf("| C%d:%.4f  |",i+1,a_d->BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
      printf("\r\n\t------------------------------------------------------------------------------------------------------------------------------\r\n\t");
      for (int i=a_d->BMS_IC[0].ic_reg.cell_channels/2; i<a_d->BMS_IC[0].ic_reg.cell_channels; i++)
      {
        printf("| C%d:%.4f |",i+1,a_d->BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
      printf("\r\n\t------------------------------------------------------------------------------------------------------------------------------\r\n");
    }
    else
    {
      printf(" Cells, ");
      for (int i=0; i<a_d->BMS_IC[0].ic_reg.cell_channels; i++)
      {
        printf("%f,",a_d->BMS_IC[current_ic].cells.c_codes[i]*0.0001);
      }
    }
  }
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
      printf(" IC %d",current_ic+1);
      for (int i=0; i < 5; i++)
      {
        printf(" GPIO-%d:%f,",i+1,a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }

      for (int i=6; i < 10; i++)
      {
        printf(" GPIO-%d:%f,",i+1,a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }

      printf(" Vref2:%f\r\n OV/UV Flags : %x\r\n",a_d->BMS_IC[current_ic].aux.a_codes[5]*0.0001,(uint8_t)a_d->BMS_IC[current_ic].aux.a_codes[11]);
    }
    else
    {
      //Serial.print(" AUX, ");
      printf(" AUX, ");

      for (int i=0; i < 12; i++)
      {
        printf("%f,",(uint8_t)a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }
    }
  }
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

    printf(" IC %d SOC:%f, Itemp:%f, VregA:%f, VregD:%f\r\n",current_ic+1,a_d->BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30,itmp,a_d->BMS_IC[current_ic].stat.stat_codes[2]*0.0001,a_d->BMS_IC[current_ic].stat.stat_codes[3]*0.0001);

    printf(" OV/UV Flags:, %.02x, %.02x, %.02x\tMux fail flag: %.02x\tTHSD:, %.02x\r\n",a_d->BMS_IC[current_ic].stat.flags[0],a_d->BMS_IC[current_ic].stat.flags[1],a_d->BMS_IC[current_ic].stat.flags[2],a_d->BMS_IC[current_ic].stat.mux_fail[0],a_d->BMS_IC[current_ic].stat.thsd[0]);
  }
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
      printf(" IC %d",current_ic+1);
      for (int i=0; i < 2; i++)
      {
        printf(" GPIO-%d:%f,",i+1,a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }
    }
    else
    {
      printf("AUX, ");

      for (int i=0; i < 12; i++)
      {
        printf("%f,",a_d->BMS_IC[current_ic].aux.a_codes[i]*0.0001);
      }
    }
  }
  printf("\r\n");
}

/*!****************************************************************************
  \brief Prints Status voltage codes for SOC onto the serial port
 *****************************************************************************/
void print_sumofcells(void)
{
  for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++)
  {
    printf(" IC %d SOC:%f,",current_ic+1,a_d->BMS_IC[current_ic].stat.stat_codes[0]*0.0001*30);
  }
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
    printf("Cell \r\n");
    }
  else if(adc_reg==2)
  {
    printf("Aux \r\n");
    }
  else if(adc_reg==3)
  {
    printf("Stat \r\n");
    }
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
    printf("Aux \r\n");
    }
  else if(adc_reg==3)
  {
    printf("Stat \r\n");
    }

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
      printf("No Opens Detected on IC %d\r\n",current_ic+1);
    }
    else
    {
      printf("There is an open wire on IC %d Channel: %ld\r\n",current_ic+1,a_d->BMS_IC[current_ic].system_open_wire);
    }
  }
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
      printf("\r\n%d : PEC Errors Detected on IC%d",a_d->BMS_IC[current_ic].crc_count.pec_count,current_ic+1);
  }
  printf("\r\n");
}

/*!****************************************************************************
  \brief prints data which is written on PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_wrpwm(void)
{
  int pwm_pec;

  printf("Written PWM Configuration: \r\n");
  for (uint8_t current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    printf("IC %d",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
     printf(", %.02x",a_d->BMS_IC[current_ic].pwm.tx_data[i]);
    }
    pwm_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].pwm.tx_data[0]);
    printf(", Calculated PEC: %.02x, %.02x\r\n",(uint8_t)(pwm_pec>>8),(uint8_t)(pwm_pec));
  }
}

/*!****************************************************************************
  \brief Prints received data from PWM register onto the serial port
  @return void
 *****************************************************************************/
void print_rxpwm(void)
{
  printf("Received pwm Configuration:\r\n");
  for (uint8_t current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    printf("IC %d",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
     printf(", %.02x",a_d->BMS_IC[current_ic].pwm.rx_data[i]);
    }
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

  printf("Written Data in Sctrl register: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    printf(" IC: %d Sctrl register group:",current_ic+1);
    for(int i = 0; i < 6; i++)
    {
      printf(", %.02x",a_d->BMS_IC[current_ic].sctrl.tx_data[i]);
    }

    sctrl_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].sctrl.tx_data[0]);
    printf(", Calculated PEC: %.02x, %.02x\r\n",(uint8_t)(sctrl_pec>>8),(uint8_t)(sctrl_pec));
  }
}

/*!****************************************************************************
  \brief prints data which is read back from S Control register
  @return void
 *****************************************************************************/
void print_rxsctrl(void)
{
   printf("Received Data:\r\n");
  for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
  {
    printf(" IC %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
    printf(", %.02x",a_d->BMS_IC[current_ic].sctrl.rx_data[i]);
    }

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

  printf(" PWM/S control register group B: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
      if(type == 1)
      {
        printf(" IC %d, %.02x, %.02x, %.02x",current_ic+1,a_d->BMS_IC[current_ic].pwmb.tx_data[0],a_d->BMS_IC[current_ic].pwmb.tx_data[1],a_d->BMS_IC[current_ic].pwmb.tx_data[2]);

        psb_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].pwmb.tx_data[0]);
        printf(", Calculated PEC: %.02x, %.02x\r\n",(uint8_t)(psb_pec>>8),(uint8_t)(psb_pec));
      }
      else if(type == 2)
      {
        printf(" IC %d\r\n",current_ic+1);
        psb_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].sctrlb.tx_data[0]);
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
  printf(" PWM/S control register group B:\r\n");
  if(type == 1)
  {
      for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
      {
        printf(" IC: %d \r\n",current_ic+1);
       printf(" %.02x, %.02x, %.02x, Received PEC: %.02x, %.02x\r\n",a_d->BMS_IC[current_ic].pwmb.rx_data[0],a_d->BMS_IC[current_ic].pwmb.rx_data[1],a_d->BMS_IC[current_ic].pwmb.rx_data[2],a_d->BMS_IC[current_ic].pwmb.rx_data[6],a_d->BMS_IC[current_ic].pwmb.rx_data[7]);

      }
  }
   else if(type == 2)
  {
      for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
      {
        printf(" IC: %d\r\n",current_ic+1);
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

  printf("Written Data in COMM Register: \r\n");
  for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
  {
    printf(" IC- %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      printf(", %x",a_d->BMS_IC[current_ic].com.tx_data[i]);
    }
    printf(", Calculated PEC: ");
    comm_pec = pec15_calc(6,&a_d->BMS_IC[current_ic].com.tx_data[0]);
    printf("%x",comm_pec>>8);
    printf(", %x\r\n",comm_pec);
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
    printf(" IC- %d",current_ic+1);

    for(int i = 0; i < 6; i++)
    {
      printf(", %x",a_d->BMS_IC[current_ic].com.rx_data[i]);
    }
    printf(", Received PEC: %x, %x\r\n",a_d->BMS_IC[current_ic].com.rx_data[6],a_d->BMS_IC[current_ic].com.rx_data[7]);
  }
}

/*!********************************************************************
  \brief Function to check the Mute bit in the Configuration Register
   @return void
**********************************************************************/

void init_app_data_bmsp(app_data *app_data_init)
{
	a_d = app_data_init;
}
