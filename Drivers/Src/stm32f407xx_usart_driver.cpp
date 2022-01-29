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
		USART1->BRR = BAUD_9600;
		USART1->CR1 |= 1 << 2 | 1 << 3 | 1 << 5 | 1 << 13;
	}
	else if(this->_usart_type == USART_3)
	{
		USART3_PCCK_EN();
		USART3->BRR = BAUD_9600;
		USART3->CR1 |= 1 << 2 | 1 << 3 | 1 << 5 | 1 << 13;
		Gpio::PinConfig_t transmitPin(PIN_10, ALT, NO_PUPD, AF7);
		Gpio::PinConfig_t recievePin(PIN_11, ALT, NO_PUPD, AF7);
		Gpio::Handler_t transmitHandler(GPIOC, &transmitPin);
		Gpio::Handler_t receiveHandler(GPIOC, &recievePin);
		Gpio::Init(&transmitHandler);
		Gpio::Init(&receiveHandler);
	}
}

void Usart::putc(char c)
{
	if(this->_usart_type == USART_1)
	{
		//while transmit data register NOT empty
		while(!(USART1->SR & TXE)){};
		USART1->DR = c;
	}
	if(this->_usart_type == USART_3)
	{
		while(!(USART3->SR & TXE)){};
		USART3->DR = c;
	}
}

void Usart::puts(const char *s)
{
	if(this->_usart_type == USART_1)
	{
		if(*s)
		{
			this->putc(*s);
			this->puts(s+=1);
		}
		//while transmit complete NOT set
		while(!(USART1->SR & TC)){};
	}
	else if(this->_usart_type == USART_3)
	{
		if(*s)
		{
			this->putc(*s);
			this->puts(s+=1);
		}
		while(!(USART3->SR & TC)){};
	}

}

