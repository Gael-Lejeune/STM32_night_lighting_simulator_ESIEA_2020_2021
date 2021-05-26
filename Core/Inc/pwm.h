#include "stm32l053xx.h"

typedef struct {
	GPIO_TypeDef * gpioPort;
	uint8_t pin;
	uint8_t af;
	TIM_TypeDef* timer;
	uint8_t canal;
} PWM_TypeDef;

//Initialisation de la PWM
void PWM_init(PWM_TypeDef *pwm, GPIO_TypeDef * port, uint8_t pin, uint8_t af, TIM_TypeDef* timer, uint8_t canal);

//Definition des valeurs de fréquence de la PWM
void PWM_set(PWM_TypeDef * pwm, uint32_t HCLKFrequency, uint32_t PWMFrequency, float duty_cycle);
