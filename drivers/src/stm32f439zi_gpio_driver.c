/*
 * stm32f439zi_gpio_driver.c
 *
 *  Created on: Nov 5, 2021
 *      Author: ha
 */

#include "stm32f439zi_gpio_driver.h"



/* Peripheral Clock Setup */
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
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DIS();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DIS();
		}
	}
}

/* Init and De-Init */

/*************************************************************
 * @name	GPIO Initialize
 * @brief	Initialize GPIO pin according to the mode selected
 * @inputs
 *
 *************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp	= 0;
	uint32_t temp1	= 0;

	//1- Enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//2- Configure the mode of the GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing bits
		pGPIOHandle->pGPIOx->MODER |= temp;														//Setting bits
	}
	else
	{
		//This part we will code later	(interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FTSR)
		{
			//Configure Falling edge
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Set bit for Falling trigger selection Register
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Clear bit for Rising trigger selection Register

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RTSR)
		{
			//Configure Rising edge
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Set bit for Rising trigger selection Register
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Clear bit for Falling trigger selection Register

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFE)
		{
			//Configure Rising/Falling edges
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Set bit for Rising trigger selection Register
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);		//Set bit for Falling trigger selection Register

		}

		//Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();															//Enable peripheral clock
		temp	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;					//To select the correct Array (correct SYSCFG_EXTICR)
		temp1	= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;					//To move bits to the correct spot
		uint32_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->SYSCFG_EXTICR[temp] = portcode << (temp1 * 4);

		//Enable the EXTI interrupt delivery IMR (interrupt mask register)
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0;

	//3- Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//4- Configure the PUPD(pull up/pull down) settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing bits
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//5- Configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing bits
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//6- Configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN)
	{
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;	//Divide by 8 to select the correct array (array 0 for pins 0-7) (array 1 for pins 8-15)
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;	//Modulus by 8 to shift bits to the correct position

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2);
	}
}

/*************************************************************
 * @name	GPIO Deinitialize
 * @brief	Deinitialize GPIO pin according to the mode selected
 * @inputs
 *
 *************************************************************/
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
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if(pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

/* Data read and write */

/*************************************************************
 * @name		GPIO_ReadFromInputPin
 * @brief		Get the state of the GPIO pin (HIGH/LOW)
 * @inputs
 *
 * 	pGPIOx:		GPIOx port base address
 * 	PinNumber:	Pin number
 * @return		0 or 1 (High or Low)
 *************************************************************/
bool GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t ReturnValue;

	ReturnValue = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x01;

	return ReturnValue;
}

/*************************************************************
 * @name		GPIO_ReadFromInputPort
 * @brief		Get the state of the GPIO port (HIGH/LOW)
 * @inputs
 *
 * 	pGPIOx:		GPIOx port base address
 * @return		0 or 1 (High or Low) for all pins in the port
 *************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t ReturnValue;

	ReturnValue = (uint16_t)pGPIOx->IDR;

	return ReturnValue;
}

/*************************************************************
 * @name		GPIO_WriteToOutputPin
 * @brief		Set the the output of the GPIOx port pin to HIGH/LOW
 * @inputs
 *
 * 	pGPIOx:		GPIOx port base address
 * 	PinNumber:	pin number in GPIOx port
 * 	value:		0 or 1 (High or Low)
 *
 * @return		None
 *************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x01 << PinNumber);		//Write 1 to GPIO pin while in output mode
	}
	else
	{
		pGPIOx->ODR &= ~(0x01 << PinNumber);	//Write 0 to GPIO pin while in output mode
	}
}

/*************************************************************
 * @name		GPIO_WriteToOutputPort
 * @brief		Set the the output of the GPIOx port (all the pins) to HIGH/LOW
 * @inputs
 *
 * 	pGPIOx:		GPIOx port base address
 * 	value:		Value of all the pins in the port
 *
 * @return		None
 *************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;		//Copy the value of "Value" into ODR
}

/*************************************************************
 * @name		GPIO_ToggleOutputPin
 * @brief		Toggle the output pin between HIGH and LOW
 * @inputs
 *
 * 	pGPIOx:		GPIOx port base address
 * 	PinNumber:	Pin number
 *
 * @return		None
 *************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (0x01 << PinNumber);
}

/* IRQ Configuration and ISR handling */

/*************************************************************
 * @name		GPIO_IRQConfig
 * @brief		Select the correct vector in External interrupt/event controller (EXTI) and store it at the correct NVIC ISER register
 * @inputs
 *
 * 	IRQNumber:		GPIOx port base address
 *	EnorDi:			Enable or Disable
 *
 * @return		None
 *************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >+ 64 && IRQNumber < 96)
		{
			//Program ISER2 register		from 64 to 95
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >+ 64 && IRQNumber < 96)
		{
			//Program ICER2 register		from 64 to 95
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

/*************************************************************
 * @name		GPIO_IRQPriorityConfig
 * @brief
 * @inputs
 *
 * 	IRQPriority:	Priority of of interrupt
 *
 * @return		None
 *************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1- First lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*************************************************************
 * @name		GPIO_IRQPriorityConfig
 * @brief
 * @inputs
 *
 * 	IRQPriority:	Priority of of interrupt
 *
 * @return		None
 *************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear the EXTI PR register corresponding to the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		 //Clear pending
		EXTI->PR |= (1 << PinNumber);
	}
}
