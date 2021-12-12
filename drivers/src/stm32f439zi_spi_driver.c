/*
 * stm32f439zi_spi_driver.c
 *
 *  Created on: Nov 16, 2021
 *      Author: ha
 */


#include "stm32f439zi_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/* Peripheral Clock Setup */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDi)
{
	if(ENorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DIS();
			}
			else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DIS();
			}
			else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DIS();
			}
			else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DIS();
			}
		}
}

/* Init and De-Init */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1- Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//2- Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_BIT_POS;

	//3- Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE_BIT_POS);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE_BIT_POS);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE_BIT_POS);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY_BIT_POS);
	}

	//4- COnfigure the speed (Baud Rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_BIT_POS;

	//5- Configure DFF (Data Frame Format)
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF_BIT_POS	;

	//6- Configure CPOL (Clock Polarity)
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_BIT_POS;

	//7- Configure CPHA (Clock Phase)
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT_POS;

	//8- Configure SSM (Software Slave Management)
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM_BIT_POS	;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
			SPI1_RST();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_RST();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_RST();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_RST();
		}
}

bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*
 * Data send and receive
 * */

/*************************************************************
 * @name		SPI_SendData
 * @brief		Send data to the outside world using SPI (Data goes from Data Register to Tx Buffer to Shift Register to the outside world)
 * @inputs
 *		pSPIx:		SPIx base address
 * 		pTxBuffer:	Data to be transmitted
 * 		length:		Length of data in bytes
 *
 * @return		None
 * @note		This is a blocking call
 *************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length)
{
	while(length > 0)
	{
		//1- Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2- Check the DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF_BIT_POS))
		{
			//16 bit DFF
			//1- load the data into the DR (data register)
			pSPIx->SPI_DR = *((uint16_t *)pTxBuffer);
			length--;
			length--;						//Two bytes of data were sent
			(uint16_t *)pTxBuffer++;		//Buffer has to be incremented by 2 bytes, hence type casting to uint16_t
		}
		else
		{
			//8 bit DFF
			pSPIx->SPI_DR = *pTxBuffer;
			length--;			//One byte of data was sent
			pTxBuffer++;					//Buffer has to be incremented by 1 byte
		}
	}
}

/*************************************************************
 * @name		SPI_ReceiveData
 * @brief		Receive data from the outside world through shift register->RxBuffer->Data Register
 * @inputs
 *		pSPIx:			SPIx base address
 * 		pRxBuffer:		Data to be received
 * 		length:			Length of data
 *
 * @return		None
 *************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length)
{
	while(length > 0)
	{
		//1- Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2- Check the DFF bit in CR1
		if(pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF_BIT_POS))
		{
			//16 bit DFF
			//1- load the data from DR into RxBuffer
			*((uint16_t *)pRxBuffer) = pSPIx->SPI_DR;
			length--;
			length--;						//Two bytes of data were sent
			(uint16_t *)pRxBuffer++;		//Buffer has to be incremented by 2 bytes, hence type casting to uint16_t
		}
		else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->SPI_DR;
			length--;			//One byte of data was sent
			pRxBuffer++;					//Buffer has to be incremented by 1 byte
		}
	}
}

/*************************************************************
 * @name		SPI_PeripheralControl
 * @brief		To enable or disable the SPI peripheral after all the Configurations are done
 * @inputs
 *		pSPIx:		SPIx base address
 * 		ENorDI:		Enable or disable
 *
 * @return		None
 *************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->SPI_CR1 |=	(1 << SPI_CR1_SPE_BIT_POS);
	}
	else
	{
		pSPIx->SPI_CR1 &=	~(1 << SPI_CR1_SPE_BIT_POS);
	}
}

/*************************************************************
 * @name		SPI_SSIConfig
 * @brief		Pull the SSI bit high or low
 * @inputs
 *		pSPIx:		SPIx base address
 * 		ENorDI:		Enable or disable
 *
 * @return		None
 *************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->SPI_CR1 |=	(1 << SPI_CR1_SSI_BIT_POS);
	}
	else
	{
		pSPIx->SPI_CR1 &=	~(1 << SPI_CR1_SSI_BIT_POS);
	}
}

/*************************************************************
 * @name		SPI_SSOEConfig
 * @brief		Pull the SSOE bit high or low to have SS (slave select) managed by hardware. Requires SSM = 1
 * @inputs
 *		pSPIx:		SPIx base address
 * 		ENorDI:		Enable or disable
 *
 * @return		None
 *************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		pSPIx->SPI_CR2 |=	(1 << SPI_CR2_SSOE_BIT_POS);
	}
	else
	{
		pSPIx->SPI_CR2 &=	~(1 << SPI_CR2_SSOE_BIT_POS);
	}
}

/*
 * IRQ Configuration and ISR handling
 * */

/*************************************************************
 * @name		SPI_IRQInterruptConfig
 * @brief		Select the correct register in the NVIC peripheral in ARM Cortex M4 processor
 * @inputs
 *
 * 	IRQNumber:		Interrupt request number
 *	EnorDi:			Enable or Disable
 *
 * @return		None
 *************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
		else if (IRQNumber >= 64 && IRQNumber < 96)
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
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 register		from 64 to 95
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}

}

/*************************************************************
 * @name		SPI_IRQPriorityConfig
 * @brief
 * @inputs
 *	IRQNumber:		Interrupt request number
 * 	IRQPriority:	Priority of of interrupt
 *
 * @return		None
 *************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1- First lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*************************************************************
 * @name		SPI_SendDataWithIT
 * @brief
 * @inputs
 *	pSPIHandle:		SPI Handle
 * 	pTxBuffer:		Data to be sent
 * 	length:			length of data
 *
 * @return		State
 *************************************************************/
uint8_t SPI_SendDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length)
{
	uint8_t state = pSPIHandle->TxState;

	//This command should be executed only when the transmit buffer is empty
	if(state != SPI_BUSY_IN_TX)
	{
		//1- Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer	=	pTxBuffer;
		pSPIHandle->TxLen		=	length;

		//2- Mark the SPI state as busy in transmission so that
		//	 no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState		=	SPI_BUSY_IN_TX;

		//3- Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2	|=	(1 << SPI_CR2_TXEIE_BIT_POS);

		//4- Data Transmission will be handled by the ISR code (will implement later)
	}

	return state;
}

/*************************************************************
 * @name		SPI_ReceiveDataWithIT
 * @brief
 * @inputs
 *	pSPIHandle:		SPI Handle
 * 	pRxBuffer:		Data to be received
 * 	length:			length of data
 *
 * @return		state
 *************************************************************/
uint8_t SPI_ReceiveDataWithIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length)
{
	uint8_t state = pSPIHandle->RxState;

	//This command should be executed only when the receive buffer is empty
	if(state != SPI_BUSY_IN_RX)
	{
		//1- Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer	=	pRxBuffer;
		pSPIHandle->RxLen		=	length;

		//2- Mark the SPI state as busy in transmission so that
		//	 no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->RxState		=	SPI_BUSY_IN_RX;

		//3- Enable the RXNEIE control bit to get interrupt whenever RXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2	|=	(1 << SPI_CR2_RXNEIE_BIT_POS);

		//4- Data Transmission will be handled by the ISR code (will implement later)
	}

	return state;
}

/*************************************************************
 * @name		SPI_IRQHandling
 * @brief		To handle interrupts in SPI
 * @inputs
 *	pHandle:		SPI Handle
 *
 * @return		state
 *************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	//Check for TXE interrupt flag and Enable control bit TXEIE in CR2 (Check SPI interrupt in reference manual for a table)
	temp1 = pHandle->pSPIx->SPI_CR2	& (1 << SPI_CR2_TXEIE_BIT_POS);
	temp2 = pHandle->pSPIx->SPI_SR	& (1 << SPI_SR_TXE_BIT_POS);

	if(temp1 && temp2)
	{
		//handle txe interrupt
		spi_txe_interrupt_handle(pHandle);
	}

	//Check for RXNE interrupt flag and Enable control bit RXNEIE in CR2 (Check SPI interrupt in reference manual for a table)
	temp1 = pHandle->pSPIx->SPI_CR2	& (1 << SPI_CR2_RXNEIE_BIT_POS);
	temp2 = pHandle->pSPIx->SPI_SR	& (1 << SPI_SR_RXNE_BIT_POS);

	if(temp1 && temp2)
	{
		//handle rxne interrupt
		spi_rxne_interrupt_handle(pHandle);
	}

	//Check for OVR interrupt flag and Enable control bit ERRIE in CR2 (Check SPI interrupt in reference manual for a table)
	temp1 = pHandle->pSPIx->SPI_CR2	& (1 << SPI_CR2_ERRIE_BIT_POS);
	temp2 = pHandle->pSPIx->SPI_SR	& (1 << SPI_SR_OVR_BIT_POS);

	if(temp1 && temp2)
	{
		//handle ovr error interrupt
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

/*********************************** Helper function implementations *********************************/

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF_BIT_POS))
	{
		//16 bit DFF
		//1- load the data into the DR (data register)
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;						//Two bytes of data were sent
		(uint16_t *)pSPIHandle->pTxBuffer++;		//Buffer has to be incremented by 2 bytes, hence type casting to uint16_t
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;						//One byte of data was sent
		pSPIHandle->pTxBuffer++;					//Buffer has to be incremented by 1 byte
	}

	//End SPI communication when TxLen is zero (when there is no more data to transmit)
	if(!pSPIHandle->TxLen)
	{
		SPI_CloseTransmission(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF_BIT_POS))
	{
		//16 bit DFF
		//1- load the data into the RxBuffer from DR (data register)
		*((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)(pSPIHandle->pSPIx->SPI_DR);
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else
	{
		//8 bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen--;						//One byte of data was received
		pSPIHandle->pRxBuffer++;					//Buffer has to be incremented by 1 byte
	}

	//End SPI communication when RxLen is zero (when there is no more data to transmit)
	if(!pSPIHandle->RxLen)
	{
		SPI_CloseReception(pSPIHandle);

		//inform the application
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Clear the OVR flag -> read from SPI_DR and then from SPI_SR

	if(pSPIHandle->TxState != SPI_BUSY_IN_RX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}

	//inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE_BIT_POS);		//prevents the generation of interrupt requests
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE_BIT_POS);		//prevents the generation of interrupt requests
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->SPI_DR;
	temp = pSPIx->SPI_SR;

	(void)temp;			//to avoid warning of temp unused
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak implementation and the application may overwrite this function
}
