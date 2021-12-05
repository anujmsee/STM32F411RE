/*
 * stm32f411re_gpio_driver.c
 *
 *  Created on: Nov 21, 2021
 *      Author: Anuj
 */


#include "stm32f411re_gpio_driver.h"

/*
 *  Peripheral Clock Setup
 */

/****************************************************************************************************
 * @fn						- GPIO_PeriClockControl
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 *  Init and De-init
 */

/****************************************************************************************************
 * @fn						- GPIO_Init
 *
 * @brief					- This function
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; 			//temporary register

	//1. configure the mode of GPIO pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// it is a non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bit of the GPIO pin
		pGPIOHandle->pGPIOx->MODER |= temp;		// setting the bit
	}
	else
	{
		// it is an interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR falling trigger selection register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure the RTSR rising trigger selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure the FTSR falling trigger selection register & RTSR rising trigger selection register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR (system configuration external interrupt control register)
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		//3. enable the EXTI external interrupt delivery using IMR interrupt mask register
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bit of the GPIO pin
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;		// setting the bit

	temp = 0;

	//3. configure the pull-up/pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPupdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bit of the GPIO pin
	pGPIOHandle->pGPIOx->PUPDR |= temp;		// setting the bit

	temp = 0;

	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	// clearing the bit of the GPIO pin
	pGPIOHandle->pGPIOx->OTYPER |= temp;		// setting the bit

	temp = 0;
	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		// configure the alternate function register

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}

/****************************************************************************************************
 * @fn						- GPIO_DeInit
 *
 * @brief					- This function
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 *  Data read and write
 */

/****************************************************************************************************
 * @fn						- GPIO_ReadFromInputPin
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- 0 or 1
 *
 * @Note					- none
 *
 *****************************************************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/****************************************************************************************************
 * @fn						- GPIO_ReadFromInputPort
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/****************************************************************************************************
 * @fn						- GPIO_WriteToOutputPin
 *
 * @brief					- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]				- Base address of the GPIO peripheral
 * @param[in]				- ENABLE or DISABLE macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/


void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// write 0 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/****************************************************************************************************
 * @fn						- GPIO_WriteToOuputPort
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/****************************************************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 *  IRQ Configuration and ISR handling
 */

/****************************************************************************************************
 * @fn						- GPIO_IRQInterruptConfig
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)	//32 to 63
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ISER2 register //64 to 95
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)	//32 to 63
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)	//64 to 95
		{
			// program ICER2 register //64 to 95
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/****************************************************************************************************
 * @fn						- GPIO_IRQPriorityConfig
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. IPRx register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/****************************************************************************************************
 * @fn						- GPIO_IRQHandling
 *
 * @brief					-
 *
 * @param[in]				-
 * @param[in]				-
 *
 * @return					- none
 *
 * @Note					- none
 *
 *****************************************************************************************************/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	// clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1 << PinNumber);
	}
}










