#include "stm32l053xx.h"

//Min res 0
//Max res fff

void rv_init(ADC_TypeDef* adc, uint8_t resolution, uint8_t channel);

uint16_t rv_getValue();
