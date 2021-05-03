/*
 * stm32f07g_gpio.driver.h
 *
 *  Created on: 18 Feb 2021
 *      Author: Erlend
 */

#ifndef INC_STM32F407G_GPIO_DRIVER_H_
#define INC_STM32F407G_GPIO_DRIVER_H_

#include "stm32f407g.h"




typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;



typedef struct
{
	// pointer to hold the base address of the GPIO peripheral

	GPIO_RegDef_t *pGPIOx; 				// This holds the base address of the GPIO port which the pin belongs to
	GPIO_PinConfig_t GPIO_PinConfig;	// This holds the GPIO pin configuration settings

}GPIO_Handle_t;


// @ GPIO PIN NUMBERS

#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15





// possible GPIO_PIN_MODES
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4		// Falling edge
#define GPIO_MODE_IT_RT		5		// Rising edge
#define GPIO_MODE_IT_RFT	6		// Rising edge trigger


#define GPIO_OP_TYPE_PP		0		// output push pull
#define GPIO_OP_TYPE_OD		1		// output open drain


#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3


#define GPIO_PIN_NO_PUPD		0	// No PIN pull-up
#define GPIO_PIN_PU				1	// PIN Pull-up
#define GPIO_PIN_PD				2	// PIN Pull-down



// APIs

int GPIO_BASEADDR_TO_CODE(GPIO_RegDef_t *pGPIOx);


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);			// Enable or disable Peripheral Clock


void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);										// 16 pins
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandler(uint8_t PinNumber);




#endif /* INC_STM32F407G_GPIO_DRIVER_H_ */
