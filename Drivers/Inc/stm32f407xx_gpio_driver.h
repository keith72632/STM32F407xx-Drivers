/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 15, 2022
 *      Author: khorton
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

#define PIN_0     0
#define PIN_1     1
#define PIN_2     2
#define PIN_3     3
#define PIN_4     4
#define PIN_5     5
#define PIN_6     6
#define PIN_7     7
#define PIN_8     8
#define PIN_9     9
#define PIN_10    10
#define PIN_11    11
#define PIN_12    12
#define PIN_13    13
#define PIN_14    14
#define PIN_15    15

#define INPUT     0x00U
#define OUTPUT    0x01U
#define ALT       0x02U
#define ANALOG    0x03U
#define INT_FT    0x04U
#define INT_RT    0x05U
#define INT_RFT   0x06U

#define AF0       0
#define AF1       1
#define AF2       2
#define AF3       3
#define AF4       4
#define AF5       5
#define AF6       6
#define AF7       7
#define AF8       8
#define AF9       9
#define AF10      10
#define AF11      11
#define AF12      12
#define AF13      13
#define AF14      14
#define AF15      15
#define NO_AF     0x00

#define NO_PUPD   0x00
#define PULL_UP   0x01
#define PULL_DOWN 0x02

/*************************************************************************************************************
 * These macro functions are used to ascertain the binary value of mode and register, to improve readability *
 *************************************************************************************************************/
//Macros that return binary value based on mode and in number
#define MODE_INPUT_VAL(PIN_NO)      (0 << (PIN_NO*2))
#define MODE_OUTPUT_VAL(PIN_NO)     (1 << (PIN_NO*2))
#define MODE_ALT_VAL(PIN_NO)        (2 << (PIN_NO*2))

//Macros that return binary value based on mode and pin number all other GPIO registers
#define ALT_VAL(MODE_VAL, PIN_NO)   (MODE_VAL << (PIN_NO * 4))
#define ODR_VAL(PIN_NO)             (1 << PIN_NO)

#define CLR_EXTI_INT(PIN_NO)        (EXTI->PR &~(1<<PIN_NO))


namespace Gpio {
	typedef struct PinConfig{
		uint8_t GPIO_PinNumber;
		uint8_t GPIO_PinMode;
		uint8_t GPIO_PinSpeed;
		uint8_t GPIO_PinPuPdControl;
		uint8_t GPIO_PinOPType;
		uint8_t GPIO_PinAltFunMode;
		PinConfig(uint8_t pinNumber, uint8_t pinMode);
		PinConfig(uint8_t pinNumber, uint8_t pinMode, uint8_t pupd);
		PinConfig(uint8_t pinNumber, uint8_t pinMode, uint8_t pupd, uint8_t altFun);
	}PinConfig_t;

	typedef struct Handler{
		GPIO_RegDef_t *pGPIOx;
		PinConfig_t *pPinConfig;
		Handler(GPIO_RegDef_t *reg, PinConfig_t *pin);
		~Handler();
	}Handler_t;
	/***********************************************************************************
	 *                       APIs supported by this driver
	 ***********************************************************************************/
	/*Clock Control*/
	void PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

	/*Initializations*/
	void Init(Handler_t *pGPIOHandle);
	void DeInit(GPIO_RegDef_t *pGPIOx);

	/*Data read and write*/
	uint8_t ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
	uint16_t ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
	void WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);
	void WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
	void ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

	/*Interrupts*/
	void IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
	void IRQHandling(void);
	void resetEXTI(uint8_t pinNo);

}
#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
