/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 15, 2022
 *      Author: khorton
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

#define INPUT     0x00U
#define OUTPUT    0x01U
#define ALT       0x02U
#define ANALOG    0x03U

typedef struct GPIO_PinConfig{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
	GPIO_PinConfig(uint8_t pinNumber)
	{
		this->GPIO_PinNumber = pinNumber;
	}
}GPIO_PinConfig_t;

typedef struct GPIO_Handle{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t *pPinConfig;
	GPIO_Handle(GPIO_RegDef_t *reg, GPIO_PinConfig_t *pin)
	{
		this->pGPIOx = reg;
		this->pPinConfig = pin;
	}
}GPIO_Handle_t;


namespace Gpio {
	/***********************************************************************************
	 *                       APIs supported by this driver
	 ***********************************************************************************/

	/*Clock Control*/
	void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

	/*Initializations*/
	void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
	void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

	/*Data read and write*/
	uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
	uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
	void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);
	void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
	void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

	/*Interrupts*/
	void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
	void GPIO_IRQHandling(void);

}
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
