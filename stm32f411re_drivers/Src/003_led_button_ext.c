/*
 * 003_led_button_ext.c
 *
 *  Created on: Nov 26, 2021
 *      Author: Anuj
 */


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

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	/* User button logic for toggling LED based on button toggle*/
	GPIOBtn.pGPIOx = GPIOB;										// Button is on PB12 i.e. Port B and Pin 12
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;		// Assigning pin #12 to the GPIO
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;			// Button is input mode as the Pin is receiving the input
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPupdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);						// Enable peripheral clock for GPIOC

	GPIO_Init(&GPIOBtn);



	while (1)
	{
		if(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12) == BTN_PRESSED)
		{
			delay(500000/2);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}
	}
	return 0;
}
