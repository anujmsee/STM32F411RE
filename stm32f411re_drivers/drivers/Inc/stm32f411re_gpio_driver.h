/*
 * stm32f411re_gpio_driver.h
 *
 *  Created on: Nov 21, 2021
 *      Author: Anuj
 */

#ifndef INC_STM32F411RE_GPIO_DRIVER_H_
#define INC_STM32F411RE_GPIO_DRIVER_H_

#include "stm32f411re.h"

/*
 *  This is a Configuration structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;				/* Possible values from @GPIO_PIN_NUMBERS*/
	uint8_t GPIO_PinMode;				/* Possible values from @GPIO_PIN_MODES*/
	uint8_t GPIO_PinSpeed;				/* Possible values from @GPIO_PIN_SPEED*/
	uint8_t GPIO_PinPupdControl;		/* Possible values from @GPIO_PIN_PUPD*/
	uint8_t GPIO_PinOPType;				/* Possible values from @GPIO_PIN_OPTYPE*/
	uint8_t GPIO_PinAltFunMode;			/* Possible values from @GPIO_PIN_ALTFUN*/
}GPIO_PinConfig_t;

/*
 *  This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;				/* This holds the base address of the GPIO port to which the pin belongs*/
	GPIO_PinConfig_t GPIO_PinConfig;	/* This holds GPIO pin configuration settings*/
}GPIO_Handle_t;

/*
 * 	@GPIO_PIN_NUMBERS
 * 	GPIO pin numbers
 */

#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15



/*
 *  @GPIO_PIN_MODES
 * 	GPIO pin possible modes
 */

#define GPIO_MODE_IN		 		0		// GPIO mode input
#define GPIO_MODE_OUT				1		// GPIO mode output
#define GPIO_MODE_ALTFN				2		// GPIO mode alternate function
#define GPIO_MODE_ANALOG			3		// GPIO mode analog
#define GPIO_MODE_IT_FT				4		// GPIO mode input falling edge trigger
#define GPIO_MODE_IT_RT				5		// GPIO mode input rising edge trigger
#define GPIO_MODE_IT_RFT			6		// GPIO mode input rising edge and falling edge trigger


/*
 *  @GPIO_PIN_OPTYPE
 *  GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP				0		// GPIO output type push pull
#define GPIO_OP_TYPE_OD				1		// GPIO output type open drain

/*
 *  @GPIO_PIN_SPEED
 * 	GPIO pin possible output speeds
 */

#define GPIO_SPEED_LOW				0		// GPIO speed low
#define GPIO_SPEED_MEDIUM			1		// GPIO speed medium
#define GPIO_SPEED_FAST				2		// GPIO speed fast
#define GPIO_SPEED_HIGH				3		// GPIO speed high

/*
 * 	@GPIO_PIN_PUPD
 *  GPIO pin pull up & pull down configuration macros
 */

#define GPIO_NO_PUPD				0		// GPIO no pull-up/pull-down
#define GPIO_PIN_PU					1		// GPIO pull-up
#define GPIO_PIN_PD					2		// GPIO pull-down



/********************************************************************************************************************
 * 											APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 ********************************************************************************************************************/

/*
 *  Peripheral Clock Setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 *  Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 *  Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOuputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 *  IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);		// ASS uint32_t instead of uint8_t
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



#endif /* INC_STM32F411RE_GPIO_DRIVER_H_ */
