#include "pwm.h"

//Initialisation de la PWM
void PWM_init(PWM_TypeDef *pwm, GPIO_TypeDef * port, uint8_t pin, uint8_t af, TIM_TypeDef* timer, uint8_t canal){
	pwm->gpioPort=port;
	pwm->pin=pin;
	pwm->af=af;
	pwm->timer=timer;
	pwm->canal=canal;

	uint8_t nb_port;
	nb_port=(uint32_t)((uint32_t *)port - IOPPERIPH_BASE)/ (uint32_t)0x400;
	//activation de l'hologe
	RCC->IOPENR|=(1<<nb_port);

	//configuration de la pin en alternatif
	pwm->gpioPort->MODER&=~(0b11<<(pwm->pin*2));
	pwm->gpioPort->MODER|=(0b10<<(pwm->pin*2));

	if (pwm->pin <= 7){
		pwm->gpioPort->AFR[0]&=~(0b0000<<(pwm->pin*4));
		pwm->gpioPort->AFR[0]|=(pwm->af<<(pwm->pin*4)); /*AFR[0] représenteAFRL et AFR[1] AFRH*/
	} else {
		pwm->gpioPort->AFR[1]&=~(0b0000<<(pwm->pin*4));
		pwm->gpioPort->AFR[1]|=(pwm->af<<(pwm->pin*4)); /*AFR[0] représenteAFRL et AFR[1] AFRH*/
	}


	//Activation du timer spécifié
	if(pwm->timer==TIM22) {
		RCC->APB2ENR|= RCC_APB2ENR_TIM22EN;
	} else if (pwm->timer==TIM21) {
		RCC->APB2ENR|= RCC_APB2ENR_TIM21EN;
	} else if (pwm->timer==TIM6) {
		RCC->APB1ENR|= RCC_APB1ENR_TIM6EN;
	} else if (pwm->timer==TIM2) {
		RCC->APB1ENR|= RCC_APB1ENR_TIM2EN;
	}

	//Activation du bon canal
	if (pwm->canal == 1) {
		pwm->timer->CCMR1|= TIM_CCMR1_OC1M_1| TIM_CCMR1_OC1M_2;
		pwm->timer->CCMR1&= ~TIM_CCMR1_OC1M_0;
		pwm->timer->CCER|= TIM_CCER_CC1E;
	} else {
		pwm->timer->CCMR1|= TIM_CCMR1_OC2M_1| TIM_CCMR1_OC2M_2;
		pwm->timer->CCMR1&= ~TIM_CCMR1_OC2M_0;
		pwm->timer->CCER|= TIM_CCER_CC2E;
	}

}

//Definition des valeurs de fréquence de la PWM
void PWM_set(PWM_TypeDef * pwm, uint32_t HCLKFrequency, uint32_t PWMFrequency, float duty_cycle) {
	int arr = HCLKFrequency/PWMFrequency-1;
	if (arr >= 65535){
		pwm->timer->PSC=999;
		arr = HCLKFrequency/(PWMFrequency *1000)-1;
	}

	if (pwm->canal == 1) {
		pwm->timer->CCR1= (arr+1)*duty_cycle;
	} else {
		pwm->timer->CCR2= (arr+1)*duty_cycle;
	}


	pwm->timer->ARR= arr;
	pwm->timer->CR1|= 1;
}



