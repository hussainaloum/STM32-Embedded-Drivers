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

	pUSARTHandle->pUSARTx->CR3 = tempreg;

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
	uint32_t PCLKx;
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
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));	//This formula is used to move the fraction part to integer

	}
	else
	{
		//USARTDIV when over sampling is 16
		usartdiv = ((25 * PCLKx) / (4 *BaudRate));	//This formula is used to move the fraction part to integer
	}

	M_part = usartdiv / 100;

	tempreg |= M_part << 4;								//Place Mantissa bits in the correct position

	F_part = (usartdiv - (M_part * 100));				//This step here is to avoid using float

	//Calculate the final fraction part
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8_BIT_POS))
	{
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);			//add 50 to round up
	}
	else
	{
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);		//add 50 to round up
	}

	tempreg |= F_part;

	//Copy the value into the BRR register
	pUSARTx->BRR = tempreg;
}

/*************************************************************
 * @name		USART_SendData
 * @brief		Initialize USART/UART peripheral
 * @inputs
 *		pUSARTHandle:	Handle of USART/UART
 *		pTxBuffer:		Data to be sent
 *		length:			Length of data to be sent
 *
 * @return		None
 *************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *tempdata = 0;

	for(; Len > 0; Len--)
	{
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));			//Wait until Transmit data register is empty

		if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
		{
			tempdata = (uint16_t *)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*tempdata & (uint16_t)0x01FF);				//Mask bits other than the first 9

			//Check if parity was enabled or not
			if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
			{
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				pTxBuffer++;
			}
		}
		else																		//if word length is 8 bits
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}

/*************************************************************
 * @name		USART_ReceiveData
 * @brief		Initialize USART/UART peripheral
 * @inputs
 *		pUSARTHandle:	Handle of USART/UART
 *		pRxBuffer:		Data to be received
 *		length:			Length of data to be received
 *
 * @return		None
 *************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length)
{
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));		//Wait till the receive buffer is empty

	//Check the word length first
	if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
	{
		//Check for parity
		if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
		{
			*((uint16_t *)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
			pRxBuffer++;
			pRxBuffer++;
		}
		else
		{
			*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint16_t)0xFF);
			pRxBuffer++;
		}
	}
	else
	{
		//Check for parity
		if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
		{
			*pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0xFF);
			pRxBuffer++;
		}
		else
		{
			*pRxBuffer = (pUSARTHandle->pUSARTx->DR & 0x7F);
			pRxBuffer++;
		}
	}
}

/*************************************************************
 * @name		USART_GetFlagStatus
 * @brief		Check flag status
 * @inputs
 *		pUSART:			USART/UART base address
 *		FlagName:		Flag name
 *
 * @return		Flag set (1) or Flag reset (0)
 *************************************************************/
bool USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName)
{
	if(pUSARTx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*************************************************************
 * @name		USART_PeripheralControl
 * @brief		Enable or Disable USART/UART peripheral
 * @inputs
 *		pUSARTx:		USART/UART base address
 *		ENorDI:			ENABLE or DISABLE
 *
 * @return		None
 *************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pUSARTx->CR1 |=	(1 << USART_CR1_UE_BIT_POS);
	}
	else
	{
		pUSARTx->CR1 &=	~(1 << USART_CR1_UE_BIT_POS);
	}
}
