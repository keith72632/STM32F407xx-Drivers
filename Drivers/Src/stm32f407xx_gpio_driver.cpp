/*
 * stm32f407xx_gpio_driver.cpp
 *
 *  Created on: Jan 15, 2022
 *      Author: khorton
 */



#include "stm32f407xx_gpio_driver.h"
#include <stdio.h>

/************************************************************************************************
 * @function                   - GPIO_PeriClockControl
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
	 * @function                   - GPIO_Init
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

		if(_mode == ALT)
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


	};


	/************************************************************************************************
	 * @function                   - GPIO_DeInit
	 *
	 * @breif                      - This function enables or disables peripheral clock for the given GPIO port
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
		return 0;
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
		return 0;
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
		pGPIOx->ODR |= ODR_VAL(pinNumber);
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
	void RQHandling(void)
	{

	};

}
