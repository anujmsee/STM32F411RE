/*
 * main.c
 *
 *  Created on: Nov 27, 2021
 *      Author: Anuj
 */

#include <string.h>
#include "stm32f411re.h"

#define LOW										0
#define HIGH									1
#define BTN_PRESSED 							LOW

void delay(uint32_t Delay)
{
	for(uint32_t i=0; i<Delay; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GPIOBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GPIOBtn));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	/* User button logic for toggling LED based on button toggle*/
	GPIOBtn.pGPIOx = GPIOA;										// Button is on PB12 i.e. Port B and Pin 12
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;		// Assigning pin #12 to the GPIO
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;			// Button is interrupt mode for a falling edge trigger
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);						// Enable peripheral clock for GPIOB

	GPIO_Init(&GPIOBtn);

	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_9, GPIO_PIN_RESET);
	//IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);

}

void EXTI9_5_IRQHandler(void)
{
	// handle the interrupt
	delay(500000/2);
	GPIO_IRQHandling(GPIO_PIN_NO_9);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
