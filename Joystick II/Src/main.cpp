/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "7segment.hpp"
#include "nrf24l01.hpp"
#include "joystick.hpp"
#include "usbd_cdc_if.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

Joystick joystick;
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	joystick.Key_Changed();
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	joystick.Time_Passed(htim);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
	HAL_Delay(2000);
	//USBD_Interface_fops_FS.Init();
	NRF nrf;
	nrf.CEPort = GPIOB;
	nrf.CEPin = GPIO_PIN_7;
	nrf.CSPort = GPIOB;
	nrf.CSPin = GPIO_PIN_8;
	nrf.module = &hspi1;
	joystick.init(&nrf,&hadc1,&htim1,ADC_CHANNEL_3,ADC_CHANNEL_2,ADC_CHANNEL_1,ADC_CHANNEL_0);
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	joystick.changeRobot(0x08);
  while (1)
  {
		///*
		unsigned char str[100];
		unsigned int length = 0;
		joystick.setX((joystick.getX() * joystick.speedConstant) * -1);
		joystick.setY(joystick.getY() * joystick.speedConstant);
		signed int Vw = joystick.getW() / 2;
		if (Vw < 500 && Vw > 500)
			Vw = 0;
		joystick.setW(Vw * 4);
		joystick.update();
		length = sprintf((char *)str,"Signal = %d \t M0 = %d \t M1 = %d \t M2 = %d \t M3 = %d \r\n",joystick.response[15],joystick.response[10] * 256 + joystick.response[11],joystick.response[12] * 256 + joystick.response[13],joystick.response[14] * 256 + joystick.response[15],joystick.response[16] * 256 + joystick.response[17]);
		CDC_Transmit_FS(str,length);
		//*/
		/*
		unsigned char str[100];
		unsigned int length = 0;
		unsigned char keys[13];
		keys[0] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10);
		keys[1] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9);
		keys[2] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
		keys[3] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
		keys[4] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
		keys[5] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
		//keys[6] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
		keys[7] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
		keys[8] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
		keys[9] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
		keys[10] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		keys[11] = (unsigned char) HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		keys[12] = (unsigned char) HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
		length = sprintf((char *)str,"1 = %d\t 2 = %d\t 3 = %d\t 4 = %d\t 5 = %d\t 6 = %d\t 8 = %d\t 9 = %d\t 10 = %d\t 11 = %d\t 12 = %d\t 13 = %d\t\r\n",keys[0],keys[1],keys[2],keys[3],keys[4],keys[5],keys[7],keys[8],keys[9],keys[10],keys[11],keys[12]);
		CDC_Transmit_FS(str,length);
		*/
		//X = 9
		//square = 8
		//circle = 10
		//triangle = 11
		//right = 5
		//left = 4
		//down = 6
		//up = 3
		//L2 = 1
		//L1 = 2
		//R1 = 12
		//R2 = 13
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
