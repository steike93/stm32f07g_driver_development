/*
 * stm32f407g.h
 *
 *  Created on: Feb 11, 2021
 *      Author: Erlen
 */

#include <stdint.h>

#ifndef INC_STM32F407G_H_
#define INC_STM32F407G_H_

#define __vo volatile					// volatile keyword tells the compiler that the value of the variable may change over time, without any action taken
										// place by the nearby code.

/*************************************************** START: Processor Specific Details ********************************************************************
 *
 *  ARM Cortex Mx Processor NVIC ISERx register Addresses.  Nested vector interrupt controller.
 *
 */

// NVIC ISERx register addresses. These register enable interrupt, and shows which interrupts are enables.

#define NVIC_ISER0 ((__vo uint32_t*)  0xE000E100)
#define NVIC_ISER1 ((__vo uint32_t*)  0XE000E104)
#define NVIC_ISER2 ((__vo uint32_t*)  0XE000E108)
#define NVIC_ISER3 ((__vo uint32_t*)  0x0000E10C)


// Arm cortex mx processor NVIC ICERx register addresses. These register disables the interrupt, and shows which interrupts ar eenabled.

#define NVIC_ICER0 ((__vo uint32_t*)  0xE000E180)
#define NVIC_ICER1 ((__vo uint32_t*)  0xE000E184)
#define NVIC_ICER2 ((__vo uint32_t*)  0xE000E188)
#define NVIC_ICER3 ((__vo uint32_t*)   0xE000E18C)

#define NVIC_PR_BASE_ADDR ((__vo uint32_t*) 0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 4



/*
 * Base addresses of Flash and SRAM memories
 *
 */

#define FLASH_BASEADDR	 			0x08000000U
#define SRAM1_BASEADDR	 			0x20000000U
#define SRAM2_BASEADDR				0x2001C000U			// SRAM1 + 1024*112KB
#define ROM							0x1FFF0000U			// system memory
#define SRAM SRAM1_BASEADR			SRAM1_BASEADDR


#define RCC_BASEADDR				(uint32_t*)0x40023800

#define PERIPH_BASEADDR				0x40000000U			// addresses need to be unsigned.
#define APB1PERIPH_BASE				PERIPH_BASEADDR
#define APB2PERIPH_BASE				0x40010000U
#define AHB1PERIPH_BASE				0x40020000U
#define AHB2PERIP_BASE				(uint32_t*)0x50000000U

#define GPIOA_BASEADDR				(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASE + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASE + 0x2000)




#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)

#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x0300)
#define USART6_BASEADDR				(APB2PERIPH_BASE + 0x1400)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x1000)
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE + 0x3800)

#define SPI_CR1						(SPI1_BASEADDR)
#define SPI_CR2						(SPI1_BASSEADDR + 0x04)
#define SPI_SR						(SPI1_BASEADDR  + 0x08)
#define SPI_DR						(SPI1_BASSEADDR + 0x0C)
#define SPI_CRCPR					(SPI1_BASEADDR  + 0x10)
#define SPI_SPI_RXCRCR				(SPI1_BASSEADDR + 0x14)
#define SPI_TXCRCR					(SPI1_BASEADDR  + 0x18)
#define SPI_I2SCFGR					(SPI1_BASSEADDR + 0x1C)
#define SPI_I2SPR					(SPI1_BASSEADDR + 0x20)





typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];			// AFRL, AFRH
} GPIO_RegDef_t	;


// Peripheral register for EXTI

typedef struct{
	__vo uint32_t IMR;				// address offset 0x00
	__vo uint32_t EMR;				// address offset 0x04
	__vo uint32_t RTSR;				// address offset 0x08
	__vo uint32_t FTSR;				// address offset 0x0C
	__vo uint32_t SWIER;			// address offset 0x10
	__vo uint32_t PR;				// address offset 0x14
} EXTI_RegDef_t	;


typedef struct{
	__vo uint32_t MEMRMP;				// address offset 0x00
	__vo uint32_t PMC;					// address offset 0x04
	__vo uint32_t EXTICR[4];			// address offset 0x08 - 0x14
		 uint32_t RESERVED1[2];			// address offset 0x18
	__vo uint32_t CMPCR;				// address offset 0x20
uint32_t      RESERVED2[2];             // Reserved, 0x24-0x28
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t	;



#define GPIOA				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/* Clock enable for GPIOx
 *
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0)) 	// controls the clock for GPIOA
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))		// controls the clock for GPIOB
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2)) 	// controls the clock for GPIOC
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))		// controls the clock for GPIOD
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4)) 	// controls the clock for GPIOE
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))		// controls the clock for GPIOF
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6)) 	// controls the clock for GPIOG
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))		// controls the clock for GPIOH
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8)) 	// controls the clock for GPIOI



/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= 1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= 1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= 1 << 22))

/*
 * Clock enable for SPIx peripherals
 */


#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->AP2ENR  |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->AP2ENR  |= (1 << 20))
#define SPI6_PCLK_EN()		(RCC->AP2ENR  |= (1 << 21))


/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		(RCC->AP2ENR)  |= (1 << 4)
#define USART2_PCLK_EN()		(RCC->AP1ENR)  |= (1 << 17)
#define USART3_PCLK_EN()		(RCC->AP1ENR)  |= (1 << 18)
#define UART4_PCLK_EN()			(RCC->AP1ENR)  |= (1 << 19)
#define UART5_PCLK_EN()			(RCC->AP2ENR)  |= (1 << 20)
#define UART6_PCLK_EN()			(RCC->AP2ENR)  |= (1 << 5)


/*
 * Clock enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()		(RCC->APB2ENR |= (1 << 14))




#define RCC_APB2RSTR_EN()		(RCC->APB2RSTR |= (1 << 14))



/*
 *  Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &=  ~(1 << 0))     // controls the clock for GPIOA
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))		// controls the clock for GPIOB
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2)) 	// controls the clock for GPIOC
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))		// controls the clock for GPIOD
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4)) 	// controls the clock for GPIOE
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))		// controls the clock for GPIOF
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6)) 	// controls the clock for GPIOG
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))		// controls the clock for GPIOH
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8)) 	// controls the clock for GPIOI
#define GPIOJ_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 9))		// controls the clock for GPIOJ
#define GPIOK_PCLK_DI()		(RCC->AHB1ENR |= (1 << 10)) 	// controls the clock for GPIOK


/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define SPI2_PCLK_DI()		(RCC->APB1ENR) &= ~(1 << 14)
#define SPI3_PCLK_DI()		(RCC->APB1ENR) &= ~(1 << 15)
#define SPI4_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 13)
#define SPI5_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 20)
#define SPI6_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 21)
/*
 * Clock disable for SPIx peripherals
 */


#define SPI1_PLCK_DI()		(RCC->APB1ENR) &= ~(1 << 12)
#define SPI2_PCLK_DI()		(RCC->APB1ENR) &= ~(1 << 14)
#define SPI3_PCLK_DI()		(RCC->APB1ENR) &= ~(1 << 15)
#define SPI4_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 13)
#define SPI5_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 20)
#define SPI6_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 21)

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		(RCC->AP2ENR)  &= ~(1 << 4)
#define USART2_PCLK_DI()		(RCC->AP1ENR)  &= ~(1 << 17)
#define USART3_PCLK_DI()		(RCC->AP1ENR)  &= ~(1 << 18)
#define UART4_PCLK_DI()			(RCC->AP1ENR)  &= ~(1 << 19)
#define UART5_PCLK_DI()			(RCC->AP2ENR)  &= ~(1 << 20)
#define UART6_PCLK_DI()			(RCC->AP2ENR)  &= ~(1 << 5)



/*
 * Clock disable Macros for SYSCFG peripherals
 */


#define RCC_APB2RSTR_DI()		(RCC->APB2RSTR &= ~(1 << 14))


typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;


#define RESERVED0 0x1C
#define RESERVED1 0X28
#define RESERVED2 0X2C
#define RESERVED4 0x3C
#define RESERVED5 0x48
#define RESERVED6 0x4C
#define RESERVED7 0x5C
#define RESERVED8 0x68
#define RESERVED9 0x6C
#define RESERVED10 0x78
#define RESERVED11 0x7C


// Macros to reset GPIOx peripherals

#define GPIOA_REG_RESET()		do{((RCC->AHB1RSTR)  |= (1 << 0)); 	((RCC->AHB1RSTR)  &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()		do{((RCC->AHB1RSTR)  |= (1 << 1)); 	((RCC->AHB1RSTR)  &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{((RCC->AHB1RSTR)  |= (1 << 2)); 	((RCC->AHB1RSTR)  &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{((RCC->AHB1RSTR)  |= (1 << 3)); 	((RCC->AHB1RSTR)  &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{((RCC->AHB1RSTR)  |= (1 << 4)); 	((RCC->AHB1RSTR)  &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{((RCC->AHB1RSTR)  |= (1 << 5)); 	((RCC->AHB1RSTR)  &= ~(1 << 5)); }while(0)

// This macro returns a code (between 0 and 7 for a given GPIO base address (x)




#define IRQ_EXTI0					6
#define IRQ_EXTI1					7
#define IRQ_EXTI2					8
#define IRQ_EXTI3					9
#define IRQ_EXTI4					10
#define IRQ_EXT9_5					23
#define IRQ_EXTI15_10				40





// Some generic macros

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET  RESET


#endif /* INC_STM32F407G_H_ */
