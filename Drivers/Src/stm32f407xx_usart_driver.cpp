/*
 * stm32f407xx_uart_driver.cpp
 *
 *  Created on: Jan 27, 2022
 *      Author: khorton
 */

#include <stm32f407xx_usart_driver.h>

Usart::Usart(USART_t usart_type)
{
	this->_usart_type = usart_type;
}

void Usart::usart_init()
{
	if(this->_usart_type == USART_1)
	{
		USART1_PCCK_EN();
		Gpio::PinConfig_t transmitPin(PIN_9, ALT, NO_PUPD, AF7);
		Gpio::PinConfig_t recievePin(PIN_10, ALT, NO_PUPD, AF7);
		Gpio::Handler_t transmitHandler(GPIOA, &transmitPin);
		Gpio::Handler_t receiveHandler(GPIOA, &recievePin);
		Gpio::Init(&transmitHandler);
		Gpio::Init(&receiveHandler);

	}
}

