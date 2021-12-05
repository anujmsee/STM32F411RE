/*
 * stm32f411re.h
 *
 *  Created on: Nov 18, 2021
 *      Author: Anuj
 */

#ifndef INC_STM32F411RE_H_
#define INC_STM32F411RE_H_

#include <stdint.h>

/*
 *	Global define
 */

#define __vo volatile

/************************************************START: Processor Specific Details***********************************************/
 /*
 *  ARM Cortex M4 Processor NVIC (Nested Vector Interrupt Controller) ISERx register Addresses
 */

#define NVIC_ISER0							((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE000E10C)

 /*
 *  ARM Cortex M4 Processor NVIC (Nested Vector Interrupt Controller) ICERx register Addresses
 */

#define NVIC_ICER0							((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0xE000E18C)

 /*
 *  ARM Cortex M4 Processor NVIC (Nested Vector Interrupt Controller) PR register Addresses
 */

#define NVIC_PR_BASE_ADDR							((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED				4

/*
 * Base address of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U	/*Flash memory base address*/
#define SRAM1_BASEADDR						0x20000000U	/*SRAM1 memory base address*/
#define ROM_BASEADDR						0x1FFF0000U	/*ROM/SYSTEM memory base address*/
#define SRAM 								SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR					0x40010000U
#define AHB1PERIPH_BASEADDR					0x40020000U
#define AHB2PERIPH_BASEADDR					0x50000000U

/*
 * Base addresses of peripherals which are associated with AHB1 Bus
 */

#define GPIOA_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASEADDR + 0x1C00)

#define RCC_BASEADDR						(AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Base addresses of peripherals which are associated with APB1 Bus
 */

#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SP12_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define TMI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000)
#define TMI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x0400)
#define TMI4_BASEADDR						(APB1PERIPH_BASEADDR + 0x0800)
#define TMI5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)

/*
 * Base addresses of peripherals which are associated with APB2 Bus
 */

#define ADC1_BASEADDR						(APB2PERIPH_BASEADDR + 0x2000)

#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR						(APB2PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR						(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR						(APB2PERIPH_BASEADDR + 0x5000)

#define TMI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x0000)
#define TMI9_BASEADDR						(APB2PERIPH_BASEADDR + 0x4000)
#define TMI10_BASEADDR						(APB2PERIPH_BASEADDR + 0x4400)
#define TMI11_BASEADDR						(APB2PERIPH_BASEADDR + 0x4800)

#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)


/****************************************peripheral register definition structures****************************************/

/*
 *	GPIOA
 */

typedef struct
{
	//Member Elements
	__vo uint32_t MODER;						/*GPIO port mode register, 					Address offset: 0x00*/
	__vo uint32_t OTYPER;						/*GPIO port output type register, 			Address offset: 0x04*/
	__vo uint32_t OSPEEDR;						/*GPIO port output speed register, 			Address offset: 0x08*/
	__vo uint32_t PUPDR;						/*GPIO port pull-up/pull-down register, 	Address offset: 0x0C*/
	__vo uint32_t IDR;							/*GPIO port input data register, 			Address offset: 0x10*/
	__vo uint32_t ODR;							/*GPIO port output data register, 			Address offset: 0x14*/
	__vo uint32_t BSRR;							/*GPIO port bit set/reset register, 		Address offset: 0x18*/
	__vo uint32_t LCKR;							/*GPIO port configuration lock register, 	Address offset: 0x1C*/
	__vo uint32_t AFR[2];						/*AFR[0] : GPIO alternate function low register, AFR[1] : GPIO alternate function high register		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;									//Structure Member GPIO_RegDef_t

/*
 *  peripheral register definition structure for RCC
 */

typedef struct
{
	__vo uint32_t CR;							/*Clock control register, 									Address offset: 0x00*/
	__vo uint32_t PLLCFGR;						/*PLL configuration register,								Address offset: 0x04*/
	__vo uint32_t CFGR;							/*Clock configuration register,								Address offset: 0x08*/
	__vo uint32_t CIR;							/*Clock interrupt register,									Address offset: 0x0C*/
	__vo uint32_t AHB1RSTR;						/*AHB1 peripheral reset register,							Address offset: 0x10*/
	__vo uint32_t AHB2RSTR;						/*AHB2 peripheral reset register,							Address offset: 0x14*/
	uint32_t RESERVED0[2];						/*Reserved, 												Address offset: 0x18-0x1C*/
	__vo uint32_t APB1RSTR;						/*APB1 peripheral reset register, 							Address offset: 0x20*/
	__vo uint32_t APB2RSTR;						/*APB2 peripheral reset register, 							Address offset: 0x24*/
	uint32_t RESERVED1[2];						/*Reserved,													Address offset: 0x28-0x2C*/
	__vo uint32_t AHB1ENR;						/*AHB1 peripheral clock enable register,					Address offset: 0x30*/
	__vo uint32_t AHB2ENR;						/*AHB2 peripheral clock enable register,					Address offset: 0x34*/
	uint32_t RESERVED2[2];						/*Reserved,				 									Address offset: 0x38-0x3C*/
	__vo uint32_t APB1ENR;						/*APB1 peripheral clock enable register, 					Address offset: 0x40*/
	__vo uint32_t APB2ENR;						/*APB2 peripheral clock enable register, 					Address offset: 0x44*/
	uint32_t RESERVED3[2];						/*Reserved,				 									Address offset: 0x48-ox4C*/
	__vo uint32_t AHB1LPENR;					/*AHB1 peripheral clock enable in low power mode register, 	Address offset: 0x50*/
	__vo uint32_t AHB2LPENR;					/*AHB2 peripheral clock enable in low power mode register,	Address offset: 0x54*/
	uint32_t RESERVED4[2];						/*Reserved,								 					Address offset: 0x58-0x5C*/
	__vo uint32_t APB1LPENR;					/*APB1 peripheral clock enable in low power mode register, 	Address offset: 0x60*/
	__vo uint32_t APB2LPENR;					/*APB2 peripheral clock enable in low power mode register,	Address offset: 0x64*/
	uint32_t RESERVED5[2];						/*Reserved,													Address offset: 0x68-0x6C*/
	__vo uint32_t BDCR;							/*Backup domain control register, 							Address offset: 0x70*/
	__vo uint32_t CSR;							/*clock control & status register, 							Address offset: 0x74*/
	uint32_t RESERVED6[2];						/*Reserved,								 					Address offset: 0x78-0x7C*/
	__vo uint32_t SSCGR;						/*spread spectrum clock generation register, 				Address offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;					/*PLLI2S configuration register, 							Address offset: 0x84*/
	uint32_t RESERVED7;							/*Reserved,								 					Address offset: 0x88*/
	__vo uint32_t DCKCFGR;						/*Dedicated Clocks Configuration Register, 					Address offset: 0x8C*/
}RCC_RegDef_t;


/*
 *  peripheral register definition structure for EXTI
 */

typedef struct
{
	//Member Elements
	__vo uint32_t IMR;							/*, 			Address offset: 0x00*/
	__vo uint32_t EMR;							/*, 			Address offset: 0x04*/
	__vo uint32_t RTSR;							/*, 			Address offset: 0x08*/
	__vo uint32_t FTSR;							/*, 			Address offset: 0x0C*/
	__vo uint32_t SWIER;						/*, 			Address offset: 0x10*/
	__vo uint32_t PR;							/*, 			Address offset: 0x14*/

}EXTI_RegDef_t;

/*
 *  peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP; 						/*Address offset: 0x00*/
	__vo uint32_t PMC;							/*Address offset: 0x04*/
	__vo uint32_t EXTICR[4];					/*Address offset: 0x08-0x14*/
	uint32_t 	  RESERVED1[2];					/*Address offset: 0x18-0x1C*/
	__vo uint32_t CMPCR;						/*Address offset: 0x20*/
	uint32_t 	  RESERVED2[2];					/*Address offset: 0x24-0x28*/
	__vo uint32_t CFGR;							/*Address offset: 0x2C*/
}SYSCFG_RegDef_t;

/*
 *  peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA 								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 								((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 *  Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1 << 0))			//GPIOA peripheral clock enable
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1 << 1))			//GPIOB peripheral clock enable
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1 << 2))			//GPIOC peripheral clock enable
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1 << 3))			//GPIOD peripheral clock enable
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1 << 4))			//GPIOE peripheral clock enable
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1 << 7))			//GPIOH peripheral clock enable

/*
 *  Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()							(RCC->APB1ENR |= (1 << 21))			//I2C1 peripheral clock enable
#define I2C2_PCLK_EN()   						(RCC->APB1ENR |= (1 << 22))			//I2C2 peripheral clock enable
#define I2C3_PCLK_EN()							(RCC->APB1ENR |= (1 << 23))			//I2C3 peripheral clock enable

/*
 *  Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()							(RCC->APB2ENR |= (1 << 12))			//SPI1 peripheral clock enable
#define SPI2_PCLK_EN()							(RCC->APB1ENR |= (1 << 14))			//SPI2 peripheral clock enable
#define SPI3_PCLK_EN()							(RCC->APB1ENR |= (1 << 15))			//SPI3 peripheral clock enable
#define SPI4_PCLK_EN()							(RCC->APB2ENR |= (1 << 13))			//SPI4 peripheral clock enable
#define SPI5_PCLK_EN()							(RCC->APB2ENR |= (1 << 20))			//SPI5 peripheral clock enable

/*
 *  Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()						(RCC->APB2ENR |= (1 << 4))			//USART1 peripheral clock enable
#define USART2_PCLK_EN()						(RCC->APB1ENR |= (1 << 17))			//USART2 peripheral clock enable
#define USART6_PCLK_EN()						(RCC->APB2ENR |= (1 << 5))			//USART6 peripheral clock enable

/*
 *  Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()						(RCC->APB2ENR |= (1 << 14))			//SYSCFG peripheral clock enable

/*
 *  Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()							(RCC->AHB1ENR &= ~(1 << 0))			//GPIOA peripheral clock disable
#define GPIOB_PCLK_DI()							(RCC->AHB1ENR &= ~(1 << 1))			//GPIOB peripheral clock disable
#define GPIOC_PCLK_DI()							(RCC->AHB1ENR &= ~(1 << 2))			//GPIOC peripheral clock disable
#define GPIOD_PCLK_DI()							(RCC->AHB1ENR &= ~(1 << 3))			//GPIOD peripheral clock disable
#define GPIOE_PCLK_DI()							(RCC->AHB1ENR &= ~(1 << 4))			//GPIOE peripheral clock disable
#define GPIOH_PCLK_DI()							(RCC->AHB1ENR &= ~(1 << 7))			//GPIOH peripheral clock disable

/*
 *  Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 21))			//I2C1 peripheral clock disable
#define I2C2_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 22))			//I2C2 peripheral clock disable
#define I2C3_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 23))			//I2C3 peripheral clock disable

/*
 *  Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()							(RCC->APB2ENR &= ~(1 << 12))			//SPI1 peripheral clock disable
#define SPI2_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 14))			//SPI2 peripheral clock disable
#define SPI3_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 15))			//SPI3 peripheral clock disable
#define SPI4_PCLK_DI()							(RCC->APB2ENR &= ~(1 << 13))			//SPI4 peripheral clock disable
#define SPI5_PCLK_DI()							(RCC->APB2ENR &= ~(1 << 20))			//SPI5 peripheral clock disable

/*
 *  Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 4))			//USART1 peripheral clock disable
#define USART2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 17))			//USART2 peripheral clock disable
#define USART6_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 5))			//USART6 peripheral clock disable

/*
 *  Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 14))			//SYSCFG peripheral clock disable

/*
 *  Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()						do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()						do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()						do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()						do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()						do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()						do { (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)

/*
 *  returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)				(	(x == GPIOA)?0:\
													(x == GPIOB)?1:\
													(x == GPIOC)?2:\
													(x == GPIOD)?3:\
													(x == GPIOE)?4:\
													(x == GPIOH)?5:0 )

/*
 *  IRQ (Interrupt Request) Number of STM32F411RE MCU
 */

#define IRQ_NO_EXTI0							6
#define IRQ_NO_EXTI1							7
#define IRQ_NO_EXTI2							8
#define IRQ_NO_EXTI3							9
#define IRQ_NO_EXTI4							10
#define IRQ_NO_EXTI9_5							23
#define IRQ_NO_EXTI15_10						40

/*
 *  Macros for all the priority levels
 */

#define NVIC_IRQ_PRIO0							0
#define NVIC_IRQ_PRIO1							1
#define NVIC_IRQ_PRIO2							2
#define NVIC_IRQ_PRIO3							3
#define NVIC_IRQ_PRIO4							4
#define NVIC_IRQ_PRIO5							5
#define NVIC_IRQ_PRIO6							6
#define NVIC_IRQ_PRIO7							7
#define NVIC_IRQ_PRIO8							8
#define NVIC_IRQ_PRIO9							9
#define NVIC_IRQ_PRIO10							10
#define NVIC_IRQ_PRIO11							11
#define NVIC_IRQ_PRIO12							12
#define NVIC_IRQ_PRIO13							13
#define NVIC_IRQ_PRIO14							14
#define NVIC_IRQ_PRIO15							15


// Some generic macros

#define ENABLE 									1
#define DISABLE									0
#define SET										ENABLE
#define RESET 									DISABLE
#define GPIO_PIN_SET							SET
#define GPIO_PIN_RESET							RESET

#include "stm32f411re_gpio_driver.h"


#endif /* INC_STM32F411RE_H_ */
