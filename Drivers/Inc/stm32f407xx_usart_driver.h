/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Jan 27, 2022
 *      Author: khorton
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

#define BAUD_9600    0x0683

#define TXE          0x0080
#define TC           0x0040

typedef enum {
	USART_1,
	USART_3
}USART_t;

class Usart
{
private:
	USART_t _usart_type;
public:
	Usart(USART_t usart_type);
	void usart_init();
	void putc(char c);
	void puts(const char *s);
};



#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
