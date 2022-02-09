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

	GPIOA->MODER &= ~(1 << (PIN_5 * 2));
	GPIOA->PUPDR |= (3 << (PIN_5 * 2));
	/*ADC*/
	//End of COnversion interrupt
	ADC1->CR1 |= 1 << 5;

	NVIC_ENABLE_IRQ(IRQ_NO_ADC);

	//Sample rate
	ADC1->SMPR2 |= 7 << 15;
	/*
	 * Sequencing
	 * SQR1 == How many channels you want
	 * SQR3 == Assign channel to sequence
	 */
	ADC1->SQR3 |= 0x05;

	//Enable the ADC for the first time and set it to continuous mode
	ADC1->CR2 |= 1 << 0 | 1 << 1;

	for(int i = 0; i < 500; i++);

	//Turn on ADC for the second time to actually run it
	ADC1->CR2 |= 1 << 0;




}
