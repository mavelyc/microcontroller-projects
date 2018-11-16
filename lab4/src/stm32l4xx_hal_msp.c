/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp_template.c
  * @author  MCD Application Team
  * @version V1.7.1
  * @date    21-April-2017
  * @brief   HAL MSP module.
  *          This file template is located in the HAL folder and should be copied 
  *          to the user folder.
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
#include "stm32l4xx_hal.h"

/** @addtogroup STM32L4xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL MSP module driver
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/** @defgroup HAL_MSP_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the Global MSP.
  * @param  None
  * @retval None
  */
void HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitialize the Global MSP.
  * @param  None  
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

/**
  * @brief  Initialize the PPP MSP.
  * @param  None
  * @retval None
  */
void HAL_PPP_MspInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */ 
}

/**
  * @brief  DeInitialize the PPP MSP.
  * @param  None  
  * @retval None
  */
void HAL_PPP_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by STM32CubeMX and eventually  
            modified by the user
   */
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */


void HAL_TIM_Base_MspInit (TIM_HandleTypeDef *htim)
{
  //Enable peripherals and GPIO Clocks 
 
	__HAL_RCC_TIM3_CLK_ENABLE(); //this is defined in stm32f4xx_hal_rcc.h
	
	
  //Configure the NVIC for TIMx 
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
  
  // Enable the TIM global Interrupt 
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}



void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{ 
	//Enable peripherals and GPIO Clocks
	__HAL_RCC_TIM3_CLK_ENABLE();
    
  //Configure the NVIC for TIMx 
	HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  
  // Enable the TIM global Interrupt 
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{	
	GPIO_InitTypeDef   GPIO_InitStruct;
  
  //- Enable peripherals and GPIO Clocks 
  __HAL_RCC_TIM4_CLK_ENABLE();
    
  // Enable GPIO Port Clocks 
  __HAL_RCC_GPIOB_CLK_ENABLE();   //use PB6 pin
  
    
  // Common configuration for all channels 
  GPIO_InitStruct.Mode 			= GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull 			= GPIO_PULLUP;
  GPIO_InitStruct.Speed 		= GPIO_SPEED_HIGH; //???? 50MHz, 25Mhz...don't know what is the critics to select pin speed.
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;  //as for which AF, see table "alternate function mapping" in datasheet
																					    //P74,  table 16. 
  // Channel 2 output 
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 


}


void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef          GPIO_InitStruct;
  static DMA_HandleTypeDef  hdma_adc;
 
  /*##-1- Enable peripherals and GPIO Clocks #################################*/

	/* ADC1 Periph clock enable */
   __HAL_RCC_ADC_CLK_ENABLE();   
   /* ADC Periph interface clock configuration */
  __HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);  //???


	/* Enable DMA2 clock */
  __HAL_RCC_DMA2_CLK_ENABLE();

  
  /*##-2- Configure peripheral GPIO ##########################################*/ 
  /* ADC1 Channe6 GPIO pin, PA1 configuration */

 /* Enable GPIO clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();   // for ADC1_IN 6, the pin is PA1

  GPIO_InitStruct.Pin 		= GPIO_PIN_1;
  GPIO_InitStruct.Mode 		= GPIO_MODE_ANALOG_ADC_CONTROL; //can not be:  GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_HIGH;
	
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(ADC1_IRQn, 3, 0);		// 2,3,4?
  HAL_NVIC_EnableIRQ(ADC1_IRQn);


 
  //##-3- Configure the DMA  
	//RM0351, table 45 & 46 on page 342 shows: ADC mapped to DMA1/channel 1  or to DMA2/channel 3.

  hdma_adc.Instance 										= DMA2_Channel3; 							//1 does not work has to change it to 3 
  
  hdma_adc.Init.Request  								= DMA_REQUEST_0;
	
	hdma_adc.Init.Direction 							= DMA_PERIPH_TO_MEMORY;
  hdma_adc.Init.PeriphInc 							= DMA_PINC_DISABLE;
  hdma_adc.Init.MemInc 									= DMA_MINC_ENABLE;   				
  hdma_adc.Init.PeriphDataAlignment 		= DMA_PDATAALIGN_WORD;  
  hdma_adc.Init.MemDataAlignment 				= DMA_MDATAALIGN_WORD;
  hdma_adc.Init.Mode 										= DMA_CIRCULAR;
  hdma_adc.Init.Priority 								= DMA_PRIORITY_MEDIUM;	
 
  HAL_DMA_DeInit(&hdma_adc);
	HAL_DMA_Init(&hdma_adc);
  
	//hdma_adc.XferCpltCallback=&hdmaXferCpltCallback;
	//HAL_DMA_RegisterCallback(&hdma_adc, HAL_DMA_XFER_CPLT_CB_ID, hdmaXferCpltCallback); //was not able to init callback function field.

  __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc);

  //##-4- Configure the NVIC for DMA 
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn,1, 0);   
  HAL_NVIC_EnableIRQ( DMA2_Channel3_IRQn);

}
  
/**
  * @brief ADC MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  static DMA_HandleTypeDef  hdma_adc;
  
  /*##-1- Reset peripherals ##################################################*/
   __HAL_RCC_ADC_FORCE_RESET();
  __HAL_RCC_ADC_RELEASE_RESET();
	/* ADC Periph clock disable
   (automatically reset all ADC's) */
  __HAL_RCC_ADC_CLK_DISABLE();


}


 
 
 
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
