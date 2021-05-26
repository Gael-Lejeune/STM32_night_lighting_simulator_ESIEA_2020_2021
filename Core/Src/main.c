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
PWM_TypeDef pwm;
BUTTON_TypeDef button;

//Valeurs de la photo résistance
float prValue = 0.0;
int iprValue;

//Valeurs du potentiomètre
float potValue = 50.0;
int ipotValue;

//Luminosité (duty_cycle)
float lum = 0.5;

//Mode d'écoute (Photorésistance/potentiomètre)
uint8_t mode = 1;

//Dernier état du bouton, utilisé pour le débugage
//uint8_t last_button_state=1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Interuption de changement de mode lors de l'appuie sur le bouton
void EXTI4_15_IRQHandler() {
	if(Button_State(&button) == 0){
		LL_mDelay(20000);
	} else {
		if(mode == 1){
			mode = 0;
			change_channel(ADC1, 0);
		} else if (mode == 0){
			mode = 1;
			change_channel(ADC1, 5);
		}
	}
	EXTI->PR|= (1 << button.pin);
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
  Button_enableIRQ(&button,LL_EXTI_TRIGGER_RISING_FALLING);

  //Initialisation de la pin PWM et définition de sa valeur
  PWM_init(&pwm, GPIOB, 3, 2, TIM2, 2);
  PWM_set(&pwm, 16000000, 50, 0.5);

  //Initialisation de la résistance variable écoutée et donc du CAN, ici, la photorésistance
  rv_init(GPIOA, 5, ADC1, 12,5);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

	  //Si le mode est le premier (observation de la luminosité)
	  if (mode == 1){
		  //ADC lis la valeur de la photo résistance
		  //La valeur est remise sur 100 (en % donc)
		  prValue = ((float)rv_getValue()/(float)4095)*100;
	  } else if (mode == 0){
		  //ADC lis la valeur du potentiomètre
		  //La valeur est remise sur 100 (en % donc)
		  potValue = ((float)rv_getValue()/(float)4095)*100;

	  }

	  //Conversion des valeurs de type float en int implicitement
	  iprValue = prValue;
	  ipotValue = potValue;

	  //Définition de la lumjinosité en fonction du rapport entre les valeurs
	  if (potValue < prValue) {
		  lum = 0.0;
	  } else {
		  lum = (potValue-prValue)/100;
	  }

	  //Changement du duty_cycle de la led en pwm afin d'ajuster la luminosité de l'éclairage
	  PWM_set(&pwm, 16000000, 50, lum);

	  //Conversion des valeurs de pot et pr en chaine de caractères
	  char prChar[20];
	  itoa(iprValue, prChar, 10);
	  char potChar[20];
	  itoa(ipotValue, potChar, 10);

	  //Si les valeurs s'écrivent sur moins de 3 caractères on adapte l'affichage
	  if(!potChar[1]){
		  potChar[1] = potChar[0];
		  potChar[0] = ' ';
	  }
	  if(!prChar[1]){
		  prChar[1] = prChar[0];
		  prChar[0] = ' ';
	  }

	  //Construction de la chaine de valeur de seuil à afficher sur le LCD caractère par caractère
	  char potStr[20];
	  potStr[0]= 'S';
	  potStr[1]= 'e';
	  potStr[2]= 'u';
	  potStr[3]= 'i';
	  potStr[4]= 'l';
	  potStr[5]= ' ';
	  potStr[6]= ':';
	  potStr[7]= ' ';
	  potStr[8]= ' ';
	  potStr[9]= ' ';
	  potStr[10]= ' ';
	  potStr[11]= ' ';
	  potStr[12] = potChar[0];
	  potStr[13] = potChar[1];
	  potStr[14] = ' ';
	  if (potChar[2]) { //Si la valeur observée fait 3 caractères ou plus
		  potStr[14] = potChar[2];
	  }
	  potStr[15] = '%';

	  //Construction de la chaine de luminosité ambiante à afficher sur le LCD caractère par caractère
	  char prStr[20];
	  prStr[0] = 'L';
	  prStr[1] = 'u';
	  prStr[2] = 'm';
	  prStr[3] = 'i';
	  prStr[4] = 'n';
	  prStr[5] = 'o';
	  prStr[6] = 's';
	  prStr[7] = 'i';
	  prStr[8] = 't';
	  prStr[9] = 'e';
	  prStr[10] = ' ';
	  prStr[11] = ':';
	  prStr[12] = prChar[0];
	  prStr[13] = prChar[1];
	  prStr[14] = ' ';
	  if (prChar[2]){
		  prStr[14] = prChar[2];
	  }
	  prStr[15] = '%';


	  //Envoie des données au LCD et affichage sur celui-ci
	  Affichage_LCD(potStr, prStr);

	  //Délai de 1.5 seconde afin d'éviter une observation des valeurs trop rapprochée
	  LL_mDelay(150000);


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
