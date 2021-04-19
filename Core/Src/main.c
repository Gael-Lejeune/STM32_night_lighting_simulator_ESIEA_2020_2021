/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

#include "lcd.h"
#include "led.h"
#include "pwm.h"
#include "resistorVar.h"
#include "button.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


/* USER CODE END PV */
LED_TypeDef led;
PWM_TypeDef pwm;
BUTTON_TypeDef button;
//Valeur de la photo résistance
uint16_t prValue = 0;

//Valeur du potentiomètre
uint16_t potValue = 0;

uint8_t mode = 1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void EXTI2_3_IRQHandler() {
//		Led_toggle(&led2);
//		//effacer le flag correspodantà l’interruption en cours
//		EXTI->PR |= (1 << bouton2.pin);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */



  //activation de l'horloge pour GPIOA etGPIOB
  RCC->IOPENR|= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /*Ici la fonction LL_mDelay sera utilisée pour faire des temps d'attente en usPour cette raison on initialise le nombre de ticks nécessaire pour faire 1msà 16000 au lieu de 16000000*/

  LL_Init1msTick(16000);

  //Initialisation du LCD
  lcdinit4(); //call lcdinit4

  //Initialisation du bouton et de son gestionnaire d'interruptions
  Button_init(&button, GPIOC, 13, LL_GPIO_PULL_NO);
  Button_enableIRQ(&button,LL_EXTI_TRIGGER_RISING);


//  AlternateFunction(GPIOC, 7, 0); //Vuedansledatasheet*
//  PWM(TIM22, 2, 16000000, 50, 0.5);
  //Initialisation de la pin PWM et définition de sa valeur
  PWM_init(&pwm, GPIOC, 7, 0, TIM22, 2);
  PWM_set(&pwm, 16000000, 50, 0.5);

  //Initialisation de la LED
  Led_init(&led, GPIOA, 5);


  GPIOA->MODER &=~(0b11 << (2*5));
  GPIOA->MODER |=(0b11 << (2*5));

  //Initialisation de la résistance variable
  rv_init(ADC1, 12,5);


  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

	  if(GPIOC->IDR&(1<<7))Led_turnOn(&led);
	  else Led_turnOff(&led);

	  //Si le mode est le premier
	  if (mode == 1){
		  //ADC lis la valeur de la photo résistance
		  prValue = rv_getValue();
	  } else {
		  //ADC lis la valeur du potentiomètre
		  potValue = rv_getValue();
	  }


	  //les deux lignes qui seront affichées sur le LCD
	  float lum = (prValue/potValue)*100;
	  PWM_set(&pwm, 16000000, 50, lum);

	  char valeurChar[20];
	  itoa(prValue, valeurChar, 10);

	  //Affichage sur le LCD

	  Affichage_LCD("100", valeurChar); //call Affichage_LCD

	  LL_mDelay(300000);


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
