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


*/




/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 


char lcd_buffer[6];    // LCD display buffer

TIM_HandleTypeDef    Tim3_Handle,Tim4_Handle;
TIM_OC_InitTypeDef Tim3_OCInitStructure, Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue,Tim4_PrescalerValue, Tim2_PrescalerValue;

__O uint8_t factor = 0;
uint16_t 								TIM4Prescaler;
__IO uint16_t Tim4_CCR;  
uint16_t Tim4_Period = 14000; 

GPIO_InitTypeDef   GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);
void FullStep(void);
void HalfStep(void);
void GPIO_Init(void);
void SetCoils(uint16_t,uint16_t,uint16_t,uint16_t);

int fullStep = 1;
int halfStep = 0;
int fsCycle = 0; // counter for full step cycle
int hsCycle = 0; // counter for half step cycle
int direction = 0; // 0 -> CW, 1 -> CCW


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

	HAL_Init();

	SystemClock_Config();   //sysclock is 80Hz. HClkm apb1 an apb2 are all 80Mhz.
  
	HAL_InitTick(0x0000); // set systick's priority to the highest.

//EXTI0_Config();
	

	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_On(LED5);

	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);  
	
	BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB5");

 	GPIO_Init();
	
	Tim4_CCR=1000; //1 s for to set interuppt
	TIM4_Config();
	TIM4_OC_Config();
	
  while (1)
  {	
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

void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clock is 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;            
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;

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
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz??

void  TIM4_Config(void)
{
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 1 KHz */
  Tim4_PrescalerValue = (uint16_t) (SystemCoreClock/ 4000) - 1;
  
  /* Set TIM4 instance */
  Tim4_Handle.Instance = TIM4; 
	Tim4_Handle.Init.Period = Tim4_Period;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}

void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}




/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			
			case GPIO_PIN_1:     	//LEFT button - Toggle half or full step
				if(halfStep ==1){		//change to full step
					Tim4_CCR = 1354; 	//CCR for fullstep
					halfStep = 0;
					fullStep = 1;
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"FS");
				}
				else { 							//change to halfstep
					Tim4_CCR = 667; 	//CCR for halfstep
					halfStep = 1;	
					fullStep = 0;		
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"HS");					
				}
				__HAL_TIM_SET_COMPARE(&Tim4_Handle, TIM_CHANNEL_1, Tim4_CCR);	//set CCR
				break;
				
			case GPIO_PIN_2:    	//RIGHT button - Toggle rotation direction					 
				if (direction == 0){
					direction = 1;
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"CW");
				}
				else { 
					direction = 0;
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"CCW");
				}
				break;
						
			case GPIO_PIN_5:    	//DOWN button - Decrease speed
				if(fullStep == 1){	//for full step
					Tim4_CCR += 100;
					
				}
				else Tim4_CCR += 100;	//for half step

				BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"DEC");
				__HAL_TIM_SET_COMPARE(&Tim4_Handle, TIM_CHANNEL_1, Tim4_CCR);	//set CCR
				break;
			
			case GPIO_PIN_3:    	//UP button - Increase speed
				if(fullStep == 1){ 	//for fullstep
					Tim4_CCR -= 100;
				}
				else Tim4_CCR -= 100;				//for half step
			
				BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t*)"INC");
				__HAL_TIM_SET_COMPARE(&Tim4_Handle, TIM_CHANNEL_1, Tim4_CCR); //set CCR
				break;
					
			default:
				break;
	  } 
}

void GPIO_Init(void) //initializes 4 pins to power the stepper motor (Pins PE10-13)
{
		__HAL_RCC_GPIOE_CLK_ENABLE();
	
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStruct.Pin = GPIO_PIN_10; //PE10
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_11; //PE11
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = GPIO_PIN_12;	//PE12
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_13;	//PE13
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

//4 cycle full step sequence for 2-coil steps
void FullStep(){
	switch(fsCycle){
		case 0:
			SetCoils(1,1,0,0);
			break;
		case 1:
			SetCoils(0,1,1,0);
			break;
		case 2:
			SetCoils(0,0,1,1);
			break;
		case 3:
			SetCoils(1,0,0,1);
			break;
		default:
			//invalid state warning
			BSP_LED_On(LED4);
			break;
	}
}

//8 cycle half step sequence
void HalfStep(){
	switch(hsCycle){
		case 0:
			SetCoils(1,0,0,0);
			break;
		case 1:
			SetCoils(1,1,0,0);
			break;
		case 2:
			SetCoils(0,1,0,0);
			break;
		case 3:
			SetCoils(0,1,1,0);
			break;
		case 4:
			SetCoils(0,0,1,0);
			break;
		case 5:
			SetCoils(0,0,1,1);
			break;
		case 6:
			SetCoils(0,0,0,1);
			break;
		case 7:
			SetCoils(1,0,0,1);
			break;
		default:
			//invalid state warning
			BSP_LED_On(LED4);
			break;
	}
}

//translate 0/1 pin configurations into GPIO outputs
void SetCoils(uint16_t a, uint16_t b, uint16_t c, uint16_t d){
	if(a){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	
	if(b){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	}
	
	if(c){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
	}
	
	if(d){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	BSP_LED_Toggle(LED4);	// debugging
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32XXX_hal_tim.c for different callback function names. 
{																															//for timer4
	BSP_LED_Toggle(LED4);
	if(direction == 0){									//CCW
				if(fsCycle < 4){
				fsCycle++;
				}
				else fsCycle = 0;		//reset
				
				if(hsCycle < 8){
				hsCycle++;					
				}
				else hsCycle = 0;		//reset
		}
		else{															//CW
				if(fsCycle > 0){
					fsCycle--;	
				}
				else fsCycle = 3;		//reset
				
				if(hsCycle > 0){
					hsCycle--;
				}
				else hsCycle = 7;		//reset
		}
	
	if(fullStep == 1) {  //execute full stepping sequence
		FullStep();
		BSP_LED_Off(LED5);				//for debugging
		BSP_LED_On(LED4);
	}

	else if(halfStep == 1) {  //execute half stepping sequence
		HalfStep();
		BSP_LED_Off(LED4);				//for debugging
		BSP_LED_On(LED5);
	}
	
	else {
		BSP_LED_On(LED4);					//for debugging
		BSP_LED_On(LED5);
	}
		__HAL_TIM_SET_COUNTER(htim, 0x0000); //reset timer counter, enables Tim4_CCR to control timer
}


static void Error_Handler(void)
{
  /* Turn LED4 on */
  //BSP_LED_On(LED4);
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
