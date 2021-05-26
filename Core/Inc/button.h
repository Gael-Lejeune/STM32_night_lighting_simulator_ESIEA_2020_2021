 /******************************************************************************
  * @file           : button.h
  * @brief          : Header for button.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************/

#include "stm32l053xx.h"
//Structure du boutton
typedef struct {
	GPIO_TypeDef * gpioPort; //GPIO port of the button pin
	uint8_t pin; //Pin number
	uint8_t pull; //Button type (push/pull)
} BUTTON_TypeDef;

//Initialisation du bouton
void Button_init(BUTTON_TypeDef *button, GPIO_TypeDef * port, uint8_t pn, uint8_t pl);

//Retourne l'état du bouton spécifié
uint8_t Button_State(BUTTON_TypeDef *button);

//Active le fonctionnement par interruption du bouton
void Button_enableIRQ(BUTTON_TypeDef *button, uint8_t trigger);
