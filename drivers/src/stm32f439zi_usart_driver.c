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
 * @name		USART_SendDataWithIT
 * @brief		Initialize USART/UART peripheral
 * @inputs
 *		pUSARTHandle:	Handle of USART/UART
 *		pRxBuffer:		Data to be received
 *		length:			Length of data to be received
 *
 * @return		None
 *************************************************************/
uint8_t USART_SendDataWithIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = length;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE_BIT_POS);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE_BIT_POS);


	}

	return txstate;
}

/*************************************************************
 * @name		USART_ReceiveDataWithIT
 * @brief		Initialize USART/UART peripheral
 * @inputs
 *		pUSARTHandle:	Handle of USART/UART
 *		pRxBuffer:		Data to be received
 *		length:			Length of data to be received
 *
 * @return		None
 *************************************************************/
uint8_t USART_ReceiveDataWithIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = length;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE_BIT_POS);

	}

	return rxstate;
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

/*************************************************************
 * @name		USART_IRQInterruptConfig
 * @brief		Configure NVIC_ISERx and NVIC_ICERx registers
 * @inputs
 *		IRQNumber:		Interrupt request number (position)
 *		ENorDI:			ENABLE or DISABLE
 *
 * @return		None
 *************************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi)
	{
		if(IRQNumber < 32)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber < 32)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*************************************************************
 * @name		USART_IRQPriorityConfig
 * @brief		Configure interrupt priority registers IPRx (4-7 Cortex - M4 user guide)
 * @inputs
 *		IRQNumber:		Interrupt request number (position)
 *		IRQPriority:	Interrupt priority number
 *
 * @return		None
 *************************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//To determine the register do the following
	uint8_t IPRx = IRQNumber / 4;

	//To find the correct place for a bit in the register
	uint8_t PRx = IRQNumber % 4;

	//calculate shift number
	uint8_t shift_number = (8 * PRx) + (8 - 4);				//4 is the number of positions in IPRx

	*(NVIC_PR_BASE_ADDR + IPRx) |= (1 << shift_number);		//IPRx here is used for offset

}

/*************************************************************
 * @name		USART_IRQHandling
 * @brief		Configure interrupt handle
 * @inputs
 *		pHandle:		Interrupt handle
 *
 * @return		None
 *************************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

	uint32_t temp1, temp2;
	uint16_t *pdata;

/*************************Check for TC flag**************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC_BIT_POS);

	 //Implement the code to check the state of TCIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE_BIT_POS);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen )
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC_BIT_POS);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag**************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE_BIT_POS);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE_BIT_POS);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen -= 1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen -= 1;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE_BIT_POS);
			}
		}
	}

/*************************Check for RXNE flag**************************/


	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE_BIT_POS);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE_BIT_POS);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USARTConfig.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						pUSARTHandle->RxLen -= 1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USARTConfig.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen -= 1;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE_BIT_POS);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}

/*************************Check for CTS flag**************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS_BIT_POS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE_BIT_POS);



	if(temp1 && temp2)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS_BIT_POS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag**************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE_BIT_POS);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE_BIT_POS);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE_BIT_POS);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag**************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE_BIT_POS;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE_BIT_POS;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an API for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



/*************************Check for Error Flag**************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE_BIT_POS) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE_BIT_POS))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF_BIT_POS))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE_BIT_POS))
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}

/*************************************************************
 * @name		USART_ApplicationEventCallback
 * @brief		Configure interrupt handle
 * @inputs
 *		pHandle:		Interrupt handle
 *		event:			event
 *
 * @return		None
 *************************************************************/
__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event)
{

}
