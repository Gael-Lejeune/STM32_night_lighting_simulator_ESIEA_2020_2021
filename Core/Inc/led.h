#include "stm32l053xx.h"

typedef struct {
	GPIO_TypeDef * gpioPort;
	uint8_t pin;
} LED_TypeDef;

//Initialisation de la LED
void Led_init(LED_TypeDef *led, GPIO_TypeDef * port, uint8_t pn);

//Allumer, éteindre, et inverser l'état de la LED
void Led_turnOn(LED_TypeDef *led);
void Led_turnOff(LED_TypeDef *led);
void Led_toggle(LED_TypeDef *led);

//Renvoyer 1 si la led est allumée ou éteinte
uint8_t Led_isOn(LED_TypeDef *led);
uint8_t Led_isOff(LED_TypeDef *led);

