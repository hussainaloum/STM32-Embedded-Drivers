/*
 * stm32f439zi_usart_driver.c
 *
 *  Created on: Dec 9, 2021
 *      Author: ha
 */

#include "stm32f439zi_usart_driver.h"
#include "stm32f439zi_rcc_driver.h"

/*************************************************************
 * @name		USART_PeriClockControl
 * @brief		Enable of Disable USART/UART peripheral clock
 * @inputs
 *		pUSARTx:		SPIx base address
 * 		ENorDi:	Data to be transmitted
 *
 * @return		None
 *************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t ENorDi)
{
	if(ENorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}
		else if(pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DIS();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DIS();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DIS();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DIS();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DIS();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DIS();
		}
		else if(pUSARTx == UART7)
		{
			UART7_PCLK_DIS();
		}
		else if(pUSARTx == UART8)
		{
			UART8_PCLK_DIS();
		}
	}
}

/*************************************************************
 * @name		USART_Init
 * @brief		Initialize USART/UART peripheral
 * @inputs
 *		pUSARTHandle:		USART/UART Handle
 *
 * @return		None
 * @Note		Initialization requires configuring the following registers
 * 				USART_CR1, USART_CR2, USART_CR3 and USART_BRR
 *************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg 	=	0;

	//Enable the peripheral clock
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

/*************************USART_CR1**************************/

	//Select the mode of USART/UART, transmitter or receiver
	if(pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE_BIT_POS);
	}
	else if(pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE_BIT_POS);
	}
	else if(pUSARTHandle->USARTConfig.USART_Mode == USART_MODE_TXRX)
	{
		//Enable both transmitter and receiver
		tempreg |= (1 << USART_CR1_TE_BIT_POS);
		tempreg |= (1 << USART_CR1_RE_BIT_POS);
	}

	//Select the word length
	tempreg |= (pUSARTHandle->USARTConfig.USART_WordLength << USART_CR1_M_BIT_POS);

	//Enable or disable parity control and select whether even or odd
	if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE_BIT_POS);				//Enable Parity control
		tempreg &= ~(1 << USART_CR1_PS_BIT_POS);				//Clear bit to select even
	}
	else if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		tempreg |= (1 << USART_CR1_PCE_BIT_POS);				//Enable Parity control
		tempreg |= (1 << USART_CR1_PS_BIT_POS);					//Set bit to select odd
	}
	else if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
	{
		tempreg &= ~(1 << USART_CR1_PCE_BIT_POS);				//Disable Parity control
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

/*************************USART_CR2**************************/
	tempreg = 0;

	//Select the number of stop bits
	tempreg |= (pUSARTHandle->USARTConfig.USART_NoOfStopBits << USART_CR2_STOP_BIT_POS);

	pUSARTHandle->pUSARTx->CR2 = tempreg;

/*************************USART_CR3**************************/
	tempreg = 0;

	//configure hardware control flow
	if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE_BIT_POS);
	}
	else if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE_BIT_POS);
	}
	else if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= (1 << USART_CR3_CTSE_BIT_POS);
		tempreg |= (1 << USART_CR3_RTSE_BIT_POS);
	}
	else if(pUSARTHandle->USARTConfig.USART_HWFlowControl == USART_HW_FLOW_CTRL_NONE)
	{
		tempreg &= ~(1 << USART_CR3_CTSE_BIT_POS);
		tempreg &= ~(1 << USART_CR3_RTSE_BIT_POS);
	}

	pUSARTHandle->pUSARTx->RC3 = tempreg;

/*************************USART_BRR**************************/

	//configure baud rate (speed)
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USARTConfig.USART_Baud);
}

/*************************************************************
 * @name		USART_SetBaudRate
 * @brief		Initialize USART/UART peripheral
 * @inputs
 *		pUSARTx:		USART/UART base address
 *		BaudRate:		Selected baud rate
 *
 * @return		None
 *************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t usartdiv = 0;
	uint8_t PCLKx;
	uint32_t tempreg = 0;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Calculate USARTDIV
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8_BIT_POS))
	{
		//USARTDIV when over sampling is 8
		usartdiv = (PCLKx/(BaudRate * 8 * (2 - 1)));
	}
	else
	{
		//USARTDIV when over sampling is 16
		usartdiv = (PCLKx/(BaudRate * 8 * (2 - 0)));
	}

	M_part = usartdiv;

	tempreg |= M_part << 4;

	F_part = (usartdiv - M_part) * 100;				//This step here is to avoid using float

	//Calculate the final fraction part
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8_BIT_POS))
	{
		F_part = ((F_part * 8) + 50) & ((uint8_t)0x07);			//add 50 to round up
	}
	else
	{
		F_part = ((F_part * 16) + 50) & ((uint8_t)0x0F);		//add 50 to round up
	}

	tempreg |= F_part;

	//Copy the value into the BRR register
	pUSARTx->BRR = tempreg;
}
