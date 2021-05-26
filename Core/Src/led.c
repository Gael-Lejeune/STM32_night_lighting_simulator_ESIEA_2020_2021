#include "led.h"

//Ce fichier ne nous sert pas dans le programme mais il est un exemple de fonctionnement d'une led

void Led_init(LED_TypeDef *led, GPIO_TypeDef * port, uint8_t pn){
	led->gpioPort=port;
	led->pin=pn;

	//d terminer le num ro du port 0--> GPIOA, 1-->GPIOB, etc.
	uint8_t nb_port;
	nb_port=(uint32_t)((uint32_t *)port - IOPPERIPH_BASE)/ (uint32_t)0x400;
	//activation de l'hologe
	//2-activation de l'hologe
	RCC->IOPENR|=1<<nb_port;

	//configuration de la pin en sortie
	led->gpioPort->MODER&=~(0b11<<2*pn);
	led->gpioPort->MODER|=(0b01<<2*pn);
}


void Led_turnOn(LED_TypeDef *led){
	led->gpioPort->ODR |= (1<<led->pin);
}


void Led_turnOff(LED_TypeDef *led){
	led->gpioPort->ODR &= ~(1<<led->pin);
}

void Led_toggle(LED_TypeDef *led){
	led->gpioPort->ODR ^= (1<<led->pin);
}

uint8_t Led_isOn(LED_TypeDef *led){
	if(led->gpioPort->ODR & (1<<led->pin))
		return 1;
	else return 0;
}

uint8_t Led_isOff(LED_TypeDef *led){
	if(!(led->gpioPort->ODR & (1<<led->pin)))
		return 1;
	else return 0;
}
