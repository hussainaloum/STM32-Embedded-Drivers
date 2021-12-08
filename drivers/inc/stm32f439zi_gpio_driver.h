/*
 * stm32f439zi_gpio_driver.h
 *
 *  Created on: Nov 5, 2021
 *      Author: ha
 */

#ifndef INC_STM32F439ZI_GPIO_DRIVER_H_
#define INC_STM32F439ZI_GPIO_DRIVER_H_

#include "stm32f439zi.h"


/* This is a Configuration structure for a GPIO pin */

typedef struct
{
	uint8_t GPIO_PinNumber;				// Pin numbers from 0-15
	uint8_t GPIO_PinMode;				// Possible values from @GPIO_MODES
	uint8_t GPIO_PinSpeed;				// Possible speed values from @SPEED_VALUES
	uint8_t GPIO_PinPuPdControl;		// Pull up/pull down control @PullUP/DOWN_CONTROLS
	uint8_t GPIO_PinOPType;				// Possible output types @OUTPUT_TYPES
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


/* THis is a handle structure for GPIO pins */

typedef struct
{
	GPIO_RegDef_t 		*pGPIOx;				//This holds the base address of the GPIO port to which the pin belongs
	GPIO_PinConfig_t 	GPIO_PinConfig;
}GPIO_Handle_t;


/******* GPIO Modes @GPIO_MODES ********/

#define GPIO_MODE_INPUT		0
#define GPIO_MODE_OUTPUT	1
#define GPIO_MODE_ALT_FN	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FTSR	4
#define GPIO_MODE_IT_RTSR	5
#define GPIO_MODE_IT_RFE	6

/* GPIO pin possible output types @OUTPUT_TYPES */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/* GPIO pin output speed @SPEED_VALUES */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/* GPIO pin pull up/pull down configuration macros @PullUP/DOWN_CONTROLS */

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/**************************************
 	 APIs supported by this GPIO driver
 **************************************/

/* Peripheral Clock Setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDi);

/* Init and De-Init */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* Data read and write */
bool GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Configuration and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F439ZI_GPIO_DRIVER_H_ */
