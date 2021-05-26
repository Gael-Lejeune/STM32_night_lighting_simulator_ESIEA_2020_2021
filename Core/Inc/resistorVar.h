 /******************************************************************************
  * @file           : resistorVar.h
  * @brief          : Header for resistorVar.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************/

#include "stm32l053xx.h"

//Initialisation de la résistance variable
void rv_init(GPIO_TypeDef * port, uint8_t pn, ADC_TypeDef* adc, uint8_t resolution, uint8_t channel);

//Cette fonction nous permet de changer le canal d'écoute du CAN
void change_channel(ADC_TypeDef* adc, uint8_t channel);

//Retourne la valeur de la résistance observée
uint16_t rv_getValue();
