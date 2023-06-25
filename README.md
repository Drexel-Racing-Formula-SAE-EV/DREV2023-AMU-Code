# DREV2023-AMU-Code
Accumulator Code

STM32F407G
subcomponents
  LTC6813 - Voltage Monitoring/Balancing - Temp Reading
    LTC6820
  dhab-155 - Current Sensing
  Fan Control

  
TODO:
	Create RTOS task for constant SPI monitoring of HAL ADC to be read into a_d.hall_current
		from this start (dis)charging task will read the same value and determine if its within safe operating thresholds
		
		convert ADC value to a current value -> determined by lab testing...
			we can also use the stm32 onboard ADC for some testing -> limited to 3.3V while DHAB outputs 5v
			
		
	Update temp monitoring function -> needs to read GPIO values 1 and 2 for all 5 segments -> (14) times?
		after collecting the raw adc value update to lab calculated temp values -> make sure the temperature is below 60*C
		finally have a fan control task that takes the highest temperature value and updates the fan speed accordingly
	
	ECU CAN Package
		Minimum Voltage read for whole accumulator
		Max Voltage read for whole accumulator
		Avg voltage for entire accumulator
		
		Max Temp per segment(5)
		AVG Temp per segment(5)
		
		Current Current
		Current Limit (set by us?)
		
		Isolation Status (read from IMD)
		
		AMU Bit(if we are good to operate or not)
		
	every frame -> ID Byte - 6 Data Bytes - Checksum Byte
	total package will be about 6-8 frames

	
	