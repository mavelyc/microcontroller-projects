/**

******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

*/




/* Includes ------------------------------------------------------------------*/
#include <main.h>
#include "string.h"
#include "stdio.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  pI2c_Handle;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range
__IO uint16_t mem;
  

char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};   
char datestring[6]={0};



uint8_t wd, dd, mo, yy, ss, mm, hh; //for weekday, day, month, year, second, minute, hour

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed, push14_pressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while (>800ms)

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void RTC_Config(void);
void RTC_AlarmAConfig(void);

// Custom prototypes
void RTC_Clock_Enable(void);// Custom function that Enables the clock
void RTC_Clock_Disable(void);// Custom function that Disables the clock
void RTC_TimeShow(void);//prints the time on the LCD
void RTC_DateShow(void);//prints the date on the LCD
void PushButton_Config1(void);//Inits external push button connected to PE14
void PushButton_Config2(void);//Inits external push button connected to PE13
char weekday[20], month[20], output[20];//glabals for printing and storing
void Get_Weekday(uint8_t WDAY);//Function that turns hex value and prints weekday string
int state, Mode_SetTime, count, push13_pressed;//counters for different states
void DisplayState(int state);
void ReadEE (void);//Function used to read the EEPROM




/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	leftpressed=0;// all initializations of counters for different states
	rightpressed=0;
	uppressed=0;
	downpressed=0;
	selpressed=0;
	sel_held=0;
	push14_pressed=0;
	Mode_SetTime = 0;
	state=0;
	count=0;
	mem=memLocation;
	push13_pressed =0;
	

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
	
											
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()
	
	BSP_LCD_GLASS_Init();	
	BSP_JOY_Init(JOY_MODE_EXTI);
	PushButton_Config1();
	PushButton_Config2();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	// first display on the LCD when reset
	HAL_Delay(1000);


//configure real-time clock
	RTC_Config();
	RTC_AlarmAConfig();
	I2C_Init(&pI2c_Handle);


//the following variables are for testging I2C_EEPROM
	uint8_t readData=0x00;
	uint16_t EE_status;



/*********************Testing I2C EEPROM------------------


	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, data1);

  
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 1 X");

	HAL_Delay(1000);
	
	EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1 , data2);
	
  if(EE_status != HAL_OK)
  {
    I2C_Error(&pI2c_Handle);
  }
	
	BSP_LCD_GLASS_Clear();
	if (EE_status==HAL_OK) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 ok");
	}else
			BSP_LCD_GLASS_DisplayString((uint8_t*)"w 2 X");

	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation);

	BSP_LCD_GLASS_Clear();
	if (data1 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);


	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);
	BSP_LCD_GLASS_DisplayString((uint8_t *)"test");

*/

  /* Infinite loop */
  while (1)
  {
			//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
			//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
			//therefore, the variable "selpressed==1" can not be used to make choice here.
			if (BSP_JOY_GetState() == JOY_SEL) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>800) {
								RTC_Clock_Disable();
								BSP_LED_On(LED4);
								BSP_LED_Off(LED5);
								RTC_DateShow();
								BSP_LED_Off(LED4);
								RTC_Clock_Enable();
								break;
					}
				}
				BSP_LCD_GLASS_Clear();
			}					
//==============================================================		
			//if (uppressed==1){
				//uppressed=0;
			//}

//==============================================================					
			if (selpressed==1)  {
	
					selpressed=0;
			} 
//==============================================================		
			//if (downpressed==1){
				//downpressed = 0;
			//}

//==============================================================		 
			if (leftpressed==1) {
				if (Mode_SetTime == 1) {
					if (state == 7) state = 1; // BORDER CASE
					else state++;
					DisplayState(state);
					leftpressed=0;
				}else{
				BSP_LCD_GLASS_Clear();// When the left button is pressed when not in Mode_SetTime
				ReadEE();							// The function to read and display the EEPROM
				leftpressed=0;				// resets the button state
				}
			}			
//==============================================================			

//==============================================================							
			if (rightpressed==1) {
				if (Mode_SetTime == 1) {
					if (state == 1) state = 7; // BORDER CASE
					else state--;
					DisplayState(state);
					rightpressed=0;
				}
			}
//==============================================================			

//==============================================================						
			if (push14_pressed==1) {
				if (Mode_SetTime == 0) {
					RTC_Clock_Disable();
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"SET");
					Mode_SetTime = 1;
				}
				else {
					BSP_LCD_GLASS_Clear();
					RTC_Clock_Enable();
					Mode_SetTime = 0;
					state = 0;
					RTC_TimeStructure.Seconds = ss;
					RTC_TimeStructure.Minutes = mm;
					RTC_TimeStructure.Hours = hh;
					RTC_DateStructure.WeekDay = wd;
					RTC_DateStructure.Date = dd;
					RTC_DateStructure.Month = mo;
					RTC_DateStructure.Year = yy;
					HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
					HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
				}
				push14_pressed = 0;
			}
			
	/*Pressing the external push button connected to PE13 will write to the EEPROM, toggling an LED to ensure 
that something has been written as well as displaying the string SAVED on the LCD screen to triple check.
The memLocation is incremented each time.  The clock is disabled prior to the write and enabled after the write.			
	*/		
			if (push13_pressed==1){
				HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
				RTC_Clock_Disable();
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)"SAVED");
				HAL_Delay(1500);
				ss = RTC_TimeStructure.Seconds;
				mm = RTC_TimeStructure.Minutes;
				hh = RTC_TimeStructure.Hours;
				EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, ss);
				if(EE_status == HAL_OK)
				{
				BSP_LED_Toggle(LED4);  
				}
				HAL_Delay(10);
				memLocation++;
				EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, mm);
				HAL_Delay(10);
				memLocation++;
				EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation, hh);
				HAL_Delay(10);
				memLocation++;	
				RTC_Clock_Enable();
				push13_pressed=0;
			}
			
			
			
			
			
			if (Mode_SetTime == 1) {
				switch (state) {
					case 0:
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"SET");
						break;
					
					case 1: 
						if (uppressed == 1) {
							ss++;
							if (ss >= 60) ss = 0;
							sprintf(output,"%d",ss);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed = 0;
						}
						if (downpressed == 1) {
							ss--;
							if (ss == 255) ss = 59;
							sprintf(output,"%d",ss);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed = 0;
						}
						break;
						
						case 2: 
						if (uppressed == 1) {
							mm++;
							if (mm >= 60) mm = 0;
							sprintf(output,"%d",mm);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed = 0;
						}
						if (downpressed == 1) {
							mm--;
							if (mm == 255) mm = 59;
							sprintf(output,"%d",mm);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed = 0;
						}
						break;
						
						case 3: 
						if (uppressed == 1) {
							hh++;
							if (hh == 24) hh = 0;
							sprintf(output,"%d",hh);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed =0;
						}
						if (downpressed == 1) {
							hh--;
							if (hh == 255) hh = 23;
							sprintf(output,"%d",hh);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed =0;
						}
						break;
						
						case 4: 
						if (uppressed == 1) {
							wd++;
							if (wd == 8) wd = 1;
							sprintf(output,"%d",wd);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed =0;
						}
						if (downpressed == 1) {	
							wd--;
							if (wd == 0) wd = 7;
							sprintf(output,"%d",wd);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed = 0;
						}
						break;
						
						case 5: 
						if (uppressed == 1) {
							dd++;
							if (dd == 32) dd = 1;
							sprintf(output,"%d",dd);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed = 0;
						}
						if (downpressed == 1) {
							dd--;
							if (dd == 0) dd = 31;
							sprintf(output,"%d",RTC_DateStructure.Date);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed = 0;
						}
						break;

						case 6: 
						if (uppressed == 1) {
							mo++;
							if (mo == 13) mo = 1;
							sprintf(output,"%d",mo);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed = 0;
						}
						if (downpressed == 1) {
							mo--;
							if (mo == 0) mo = 12;
							sprintf(output,"%d",mo);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed = 0;
						}
						break;
						
						case 7: 
						if (uppressed == 1) {
							yy++;
							if (yy == 100) yy = 0;
							sprintf(output,"%d",yy);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							uppressed =0;
						}
						if (downpressed == 1) {
							yy--;
							if (yy == 255) yy = 99;
							sprintf(output,"%d",yy);
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)output);
							downpressed =0;
						}
						break;
				} // end of switch
				
				
			} //end of if		
		


	}
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */



//The function to read the EEPROM.  The clock is disabled prior to and enabled after the function.
//%02d is used to fill the 10s place of a digit even though it is a singled digit ex. 01
void ReadEE (void){
	RTC_Clock_Disable();
	hh=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-4);
	HAL_Delay(10);
	mm=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-5);
	HAL_Delay(10);
	ss=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-6);
	HAL_Delay(10);
	sprintf(output,"      TIME1=%02d%02d%02d",hh,mm,ss);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)output,1,300);
	BSP_LCD_GLASS_Clear();
	hh=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-1);
	HAL_Delay(10);
	mm=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-2);
	HAL_Delay(10);
	ss=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation-3);
	HAL_Delay(10);
	sprintf(output,"     TIME2=%02d%02d%02d",hh,mm,ss);
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)output,1,300);//displays a sentence that scrolls across the LCD
	RTC_Clock_Enable();


}
void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz


void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure;
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//****2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		
	
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;//We chose the 24 hour format
				
				RTCHandle.Init.AsynchPrediv = 0x7F;
				RTCHandle.Init.SynchPrediv = 0xFF; 
				
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
				
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
				}
	
	
	
	
	//****3.***** init the time and 
				
					
				RTC_DateStructure.Year = 0x12; // Set all the dates in Hexadecmial
				RTC_DateStructure.Month = 0x10;
				RTC_DateStructure.Date = 0x14;
				RTC_DateStructure.WeekDay = 0x04;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
  
				RTC_TimeStructure.Hours = 0;  //These are all initialized to 0
				RTC_TimeStructure.Minutes = 0; 
				RTC_TimeStructure.Seconds = 0;
				RTC_TimeStructure.TimeFormat = RTC_FORMAT_BIN;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
	  



				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to ‘1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A;
  RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL;


  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is use d, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0:     //SELECT button					
						selpressed=1;	
						break;	
			case GPIO_PIN_1:     //left button						
						leftpressed=1;
						break;
			case GPIO_PIN_2:    //right button
						rightpressed=1;			
						break;
			case GPIO_PIN_3:
						uppressed=1;						
						break;
			case GPIO_PIN_5:    //down button		
						downpressed=1;
						break;
			case GPIO_PIN_14:    //external push button						
						RTC_Clock_Disable();//disables the clock
						push14_pressed=1;
						break;	
			case GPIO_PIN_13:// external push button
						push13_pressed=1;
			default://
						//default
						break;
	  } 
}




void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)// this function calls the Alarm event everysecond through interrupts therefore updating the time
{
  BSP_LED_Toggle(LED5);
	RTC_TimeShow();
	
}

void RTC_Clock_Enable(void)// Enables the clock and shuts off LED4
{
RTC_AlarmA_IT_Enable(&RTCHandle);
BSP_LED_Off(LED5);
}

void RTC_Clock_Disable(void)// Disables the clock
{
RTC_AlarmA_IT_Disable(&RTCHandle);
}


void RTC_TimeShow(void)// Prints the time on the LCD
{
	HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
	char TimeStamp[20];
	sprintf(TimeStamp,"%02d%02d%02d",RTC_TimeStructure.Hours,RTC_TimeStructure.Minutes,RTC_TimeStructure.Seconds);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)TimeStamp);
}

void RTC_DateShow(void)// Prints the date on the LCD
{

	HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN);
	char DateStamp[50];
	Get_Weekday(RTC_DateStructure.WeekDay);
	sprintf(DateStamp,"      WDAY-%s, DATE-%d, MONTH-%d, YEAR-20%d",weekday,RTC_DateStructure.Date,RTC_DateStructure.Month,RTC_DateStructure.Year);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)DateStamp,1,300);

}

void Get_Weekday(uint8_t WDAY) {// switch case for printing the day instead of a hexadecimal representation of the date
	switch (WDAY) {
		case 0x01:
				strcpy(weekday,"MON");
				break;
		case 0x02:
				strcpy(weekday,"TUES");
				break;
		case 0x03:
				strcpy(weekday,"WED");
				break;
		case 0x04:
				strcpy(weekday,"THURS");
				break;
		case 0x05:
				strcpy(weekday,"FRI");
				break;
		case 0x06:
				strcpy(weekday,"SAT");
				break;
		case 0x07:
				strcpy(weekday,"SUN");
				break;
	}
}


void DisplayState(int state) {//displays the state of which value you choose to set as you traverse left and right
	BSP_LCD_GLASS_Clear();
	switch (state) {
		case 1: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"SEC");
			break;
		case 2: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"MIN");
			break;
		case 3: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"HOUR");
			break;
		case 4: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"WDAY");
			break;
		case 5: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"DATE");
			break;
		case 6: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"MONTH");
			break;
		case 7: 
			BSP_LCD_GLASS_DisplayString((uint8_t*)"YEAR");
			break;
	}
}

void PushButton_Config1(void)//Configured the push button the set the time
{
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_14;// init to pin14
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;//pull down for external debouncing with a capacitor
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 2, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));
}

void PushButton_Config2(void)
{
	__HAL_RCC_GPIOE_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_13;// init to pin13
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;// pulldown for external debouncing with a capacitor
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority((IRQn_Type)(EXTI15_10_IRQn), 2, 0);
	HAL_NVIC_EnableIRQ((IRQn_Type)(EXTI15_10_IRQn));
}

static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
