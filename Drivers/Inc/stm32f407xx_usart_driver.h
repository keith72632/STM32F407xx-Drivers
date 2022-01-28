/*
 * stm32f407xx_uart_driver.h
 *
 *  Created on: Jan 27, 2022
 *      Author: khorton
 */

#ifndef INC_STM32F407XX_USART_DRIVER_H_
#define INC_STM32F407XX_USART_DRIVER_H_

#include "stm32f407xx.h"

typedef enum {
	USART_1
}USART_t;

class Usart
{
private:
	USART_t _usart_type;
public:
	Usart(USART_t usart_type);
	void usart_init();
};



#endif /* INC_STM32F407XX_USART_DRIVER_H_ */
