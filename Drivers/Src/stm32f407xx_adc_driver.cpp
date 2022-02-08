/*
 * stm32f407xx_adc_driver.cpp
 *
 *  Created on: Feb 7, 2022
 *      Author: khorton
 */


#include "stm32f407xx_adc_driver.h"

void ADC_Init(void)
{
	RCC->CFGR |= 0x06 << 13;

	RCC->APB2ENR |= 1 << 8;

	RCC->AHB1ENR |= 1 << 0;

	GPIOA->MODER &= ~(1 << 10);
	GPIOA->PUPDR |= (3 << 10);

	/*ADC*/
	volatile ADC_RegDef_t *adc1 = (ADC_RegDef_t*)ADC1_BASEADDR;
	adc1->CR1 |= 1 << 5;
}
