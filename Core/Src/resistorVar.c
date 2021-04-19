#include "resistorVar.h"
#include "stm32l0xx_ll_exti.h"


void rv_init(ADC_TypeDef* adc, uint8_t resolution, uint8_t channel){
	RCC->APB2ENR|= RCC_APB2ENR_ADCEN;//Activation del’horloge
	adc->CFGR2|=(0b11<<ADC_CFGR2_CKMODE_Pos);//Mode synchronous clock

	if((adc->CR& ADC_CR_ADEN) != 0) {
		adc->CR&= ~(uint32_t)ADC_CR_ADEN;
	}
	ADC1->CR|= ADC_CR_ADCAL;//Calibration
	while((ADC1->CR& ADC_CR_ADCAL) != 0);//Attente ADCAL = 0 (fin calibration)

	adc->CFGR1|=ADC_CFGR1_CONT;//Activation du mode continue

	//Définition de la résolution
	if(resolution==12) {
		adc->CFGR1&=~(0b11<<3);
	} else if(resolution==10) {
		adc->CFGR1&=~(0b11<<3);
		adc->CFGR1|=(0b01<<3);
	} else if(resolution==8) {
		adc->CFGR1&=~(0b11<<3);
		adc->CFGR1|=(0b10<<3);
	} else if(resolution==6) {
		adc->CFGR1|=(0b11<<3);
	}
	adc->CHSELR|=(1<<channel);//Selection du canal
	adc->CR|=ADC_CR_ADEN;//Activation duconvertisseur
	adc->CR|=ADC_CR_ADSTART;//Activation du dialogue
}

uint16_t rv_getValue(){
	  while((ADC1->ISR& ADC_ISR_EOC)==0);
	  return ADC1->DR;
}



