/*
 * stm32f407xx_gpio_driver.cpp
 *
 *  Created on: Jan 15, 2022
 *      Author: khorton
 */



#include "stm32f407xx_gpio_driver.h"
#include <stdio.h>

/************************************************************************************************
 * @function                   - PeriClockControl
 *
 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]                  - Base address of the gpio peripheral
 * @param[in]                  - Enable or Disable macros
 *
 * @return                     - None
 *
 * @Note
 * */

namespace Gpio {
	PinConfig_t::PinConfig(uint8_t pinNumber, uint8_t pinMode)
	{
		this->GPIO_PinNumber = pinNumber;
		this->GPIO_PinMode = pinMode;
	}

	PinConfig_t::PinConfig(uint8_t pinNumber, uint8_t pinMode, uint8_t pupd)
	{
		this->GPIO_PinNumber = pinNumber;
		this->GPIO_PinMode = pinMode;
		this->GPIO_PinPuPdControl = pupd;
	}

	PinConfig_t::PinConfig(uint8_t pinNumber, uint8_t pinMode, uint8_t pupd, uint8_t altFun)
	{
		this->GPIO_PinNumber = pinNumber;
		this->GPIO_PinMode = pinMode;
		this->GPIO_PinPuPdControl = pupd;
		this->GPIO_PinAltFunMode = altFun;
	}

	Handler_t::Handler(GPIO_RegDef_t *reg, PinConfig_t *pin)
	{
		this->pGPIOx = reg;
		this->pPinConfig = pin;
	}

//	Handler_t::~Handler(void)
//	{
//		if(this->pGPIOx==GPIOA)
//		{
//			GPIOA_PCLK_DI();
//			GPIOA_REG_RESET();
//		}
//		else if(this->pGPIOx==GPIOB)
//		{
//			GPIOB_PCLK_DI();
//			GPIOB_REG_RESET();
//		}
//		else if(this->pGPIOx==GPIOC)
//		{
//			GPIOC_PCLK_DI();
//			GPIOC_REG_RESET();
//		}
//		else if(this->pGPIOx==GPIOD)
//		{
//			GPIOD_PCLK_DI();
//			GPIOD_REG_RESET();
//		}
//	}

	void PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
	{
		if(EnorDi)
		{
			if(pGPIOx==GPIOA)
			{
				GPIOA_PCLK_EN();
			}
			else if(pGPIOx==GPIOB)
			{
				GPIOB_PCLK_EN();
			}
			else if(pGPIOx==GPIOC)
			{
				GPIOC_PCLK_EN();
			}
			else if(pGPIOx==GPIOD)
			{
				GPIOD_PCLK_EN();
			}
		}
		else
		{
			if(pGPIOx==GPIOA) GPIOA_PCLK_DI();
			else if(pGPIOx==GPIOB) GPIOB_PCLK_DI();
			else if(pGPIOx==GPIOC) GPIOC_PCLK_DI();
			else if(pGPIOx==GPIOD) GPIOD_PCLK_DI();
		}
	};


	/*Initializations*/
	/************************************************************************************************
	 * @function                   - Init
	 *
	 * @breif                      -
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	void Init(Handler_t *pGPIOHandle)
	{
		uint32_t _pinNo, _mode;

		PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
		//GPIO Pin Mode
		_pinNo = pGPIOHandle->pPinConfig->GPIO_PinNumber;
		_mode = pGPIOHandle->pPinConfig->GPIO_PinMode;
		if(_mode == OUTPUT)
		{
			pGPIOHandle->pGPIOx->MODER |= MODE_OUTPUT_VAL(_pinNo);
		}

		else if(_mode == ALT)
		{
			uint8_t _altVal = pGPIOHandle->pPinConfig->GPIO_PinAltFunMode;
			pGPIOHandle->pGPIOx->MODER |= MODE_ALT_VAL(_pinNo);
			if(_pinNo < 8)
			{
				pGPIOHandle->pGPIOx->AFR[0] |=  ALT_VAL(_altVal, _pinNo);
			}
			else
			{
				pGPIOHandle->pGPIOx->AFR[1] |= ALT_VAL(_altVal, (_pinNo - 8));
			}
		}

		else if (_mode == INPUT)
		{
			pGPIOHandle->pGPIOx->MODER &= MODE_INPUT_VAL(_pinNo);
		}

		else if(_mode == INT_FT)
		{
			SYSCFG_PCLK_EN();

			uint8_t temp = _pinNo / 4;
			uint8_t pupd = pGPIOHandle->pPinConfig->GPIO_PinPuPdControl;

			pGPIOHandle->pGPIOx->PUPDR |= (pupd << (_pinNo *2));

			SYSCFG->EXTICR[temp] &= ~(0xf << (_pinNo *4));

			EXTI->IMR |= (1 << _pinNo);

			EXTI->FTSR |= (1 << _pinNo);
				//clear the corresonding RTSR bit
			EXTI->RTSR &= ~(1 << _pinNo);

			*NVIC_ISER0 |= 1 << IRQ_NO_EXTI0;
		}

		else if(_mode == INT_RT)
		{
			SYSCFG_PCLK_EN();

			uint8_t temp = _pinNo / 4;

			pGPIOHandle->pGPIOx->PUPDR |= (1 << (_pinNo *2));

			SYSCFG->EXTICR[temp] &= ~(0xf << (_pinNo *4));

			EXTI->IMR |= (1 << _pinNo);

			EXTI->FTSR &= ~(1 << _pinNo);
				//clear the corresonding RTSR bit
			EXTI->RTSR |= (1 << _pinNo);

			*NVIC_ISER0 |= 1 << IRQ_NO_EXTI1;
		}

		else if(_mode == INT_RFT)
		{
			SYSCFG_PCLK_EN();

			uint8_t temp = _pinNo / 4;

			pGPIOHandle->pGPIOx->PUPDR |= (1 << (_pinNo *2));

			SYSCFG->EXTICR[temp] &= ~(0xf << (_pinNo *4));

			EXTI->IMR |= (1 << _pinNo);

			EXTI->FTSR |= (1 << _pinNo);
			//clear the corresonding RTSR bit
			EXTI->RTSR |= (1 << _pinNo);

			*NVIC_ISER0 |= 1 << IRQ_NO_EXTI1;
		}


	};


	/************************************************************************************************
	 * @function                   - DeInit
	 *
	 * @breif                      - This function dinitializes the GPIO Clock
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	void DeInit(GPIO_RegDef_t *pGPIOx)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
	};


	/*Data read and write*/
	/************************************************************************************************
	 * @function                   - GPIO_ReadFromInputPin
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 * @param[in]                  - Number of pin to read from
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	uint8_t ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
	{
		return (pGPIOx->IDR >> pinNumber) & 0x00000001;
	};


	/************************************************************************************************
	 * @function                   - GPIO_ReadFromInputPort
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	uint16_t ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
	{
		return pGPIOx->IDR;
	};


	/************************************************************************************************
	 * @function                   - GPIO_WriteToOutputPin
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 * @param[in]                  - Enable or Disable macros
	 * @param[in]                  -
	 * @return                     - None
	 *
	 * @Note
	 * */
	void WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value)
	{
		uint32_t val = ODR_VAL(pinNumber);
		if(Value)
			pGPIOx->ODR |= val;
		else
			pGPIOx->ODR &= ~val;

	};


	/************************************************************************************************
	 * @function                   - GPIO_WriteToOutputPort
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 * @param[in]                  - Enable or Disable macros
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	void WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
	{
		pGPIOx->ODR |= Value;
	};


	/************************************************************************************************
	 * @function                   - GPIO_ToggleOutputPin
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 * @param[in]                  - Enable or Disable macros
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	void ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
	{
		pGPIOx->ODR ^= ODR_VAL(pinNumber);
	};


	/*Interrupts*/
	/************************************************************************************************
	 * @function                   - GPIO_IRQConfig
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 * @param[in]                  - Base address of the gpio peripheral
	 * @param[in]                  - Enable or Disable macros
	 * @param[in]                  -
	 * @return                     - None
	 *
	 * @Note
	 * */
	void IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
	{

	};


	/************************************************************************************************
	 * @function                   - GPIO_IRQHandling
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
	 *
	 *
	 * @return                     - None
	 *
	 * @Note
	 * */
	void IRQHandling(void)
	{

	};

	void resetEXTI(uint8_t pinNo)
	{
		if(EXTI->PR |= (1 << pinNo))
		{
			EXTI->PR &= ~(1 << pinNo);
		}
	};


}
