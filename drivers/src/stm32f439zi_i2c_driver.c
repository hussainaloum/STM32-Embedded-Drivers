/*
 * stm32f439zi_i2c_driver.c
 *
 *  Created on: Nov 19, 2021
 *      Author: ha
 */

#include "stm32f439zi_i2c_driver.h"
#include "stm32f439zi_rcc_driver.h"

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);


/*************************************************************
 * @name		I2C_PeripheralControl
 * @brief		Enable or disable I2C peripheral
 * @inputs
 * 		pI2Cx:		I2Cx base address
 * 		ENorDi:		Enable or Disable
 *
 * @return		None
 *************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE_BIT_POS);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}

}

/*************************************************************
 * @name		I2C_PeriClockControl
 * @brief		Enable or disable the I2C peripheral clock
 * @inputs
 * 		pI2Cx:		I2Cx base address
 * 		ENorDi:		Enable or Disable
 *
 * @return		None
 *************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDi)
{
	if(ENorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DIS();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DIS();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DIS();
		}
	}
}

/*************************************************************
 * @name		I2C_PeriClockControl
 * @brief		Enable or disable the I2C peripheral clock
 * @inputs
 * 		pI2Cx:		I2Cx base address
 * 		ENorDi:		Enable or Disable
 *
 * @return		None
 *************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

		//enable the clock for the i2cx peripheral					Check
		I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

		//ack control bit
		tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;
		pI2CHandle->pI2Cx->CR1 = tempreg;

		//configure the FREQ field of CR2							check
		tempreg = 0;
		tempreg |= RCC_GetPCLK1Value() /1000000U ;
		pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

	   //program the device own address
		tempreg = 0;
		tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
		tempreg |= ( 1 << 14);
		pI2CHandle->pI2Cx->OAR1 = tempreg;

		//CCR calculations											-->
		uint16_t ccr_value = 0;
		tempreg = 0;
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode
			ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			tempreg |= (ccr_value & 0xFFF);
		}
		else
		{
			//mode is fast mode
			tempreg |= ( 1 << 15);
			tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
			if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			else
			{
				ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
			}
			tempreg |= (ccr_value & 0xFFF);
		}
		pI2CHandle->pI2Cx->CCR = tempreg;

		//TRISE Configuration
		if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
		{
			//mode is standard mode

			tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

		}else
		{
			//mode is fast mode
			tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;

		}

		pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

}

/*************************************************************
 * @name		I2C_DeInit
 * @brief		Initialize the I2Cx peripheral
 * @inputs
 * 		pI2Cx:		I2Cx base address
 *
 * @return		None
 *************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_RST();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_RST();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_RST();
	}
}

/*************************************************************
 * @name		I2C_GetFlagStatus
 * @brief		Return the status of a flag in SR register
 * @inputs
 * 		pI2Cx:			I2Cx base address
 * 		FlagName:		Flag name
 *
 * @return		One or Zero (set or cleared)
 *************************************************************/
bool I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*************************************************************
 * @name		I2C_MasterSendData
 * @brief		Send data on the I2C peripheral
 * @inputs
 * 		pI2CHandle:		I2Cx handle
 * 		pTxbuffer:		Data to be sent
 * 		Len:			Length of data
 * 		SlaveAddrs:		Address of slave
 * 		Sr:				Disable or enable repeated start
 *
 * @return		None
 *************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddrs, uint8_t Sr)
{
	//1- Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);


	//2- Confirm start generation is completed by checking the SB flag in SR1
	//Not: until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3- Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddrs);

	//4- Confirm that address phase was completed by checking the ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5 Clear the ADDR flag according to its software sequence
	//Note: Until ADDR is cleared SCL will be stretched (pulled LOW)
	I2C_ClearADDRFlag(pI2CHandle);

	//6- Send the data until Len becomes 0
	while(Len > 0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));		//Wait till TXE is set (Register not empty)

		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7 When Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// Note: TXE=1, BTF=1 means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (pulled LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8- generate STOP condition and master ddoesn't need to wait for the completion of stop condition.
	//Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR )
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP_BIT_POS);

	I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);

}

/*************************************************************
 * @name		I2C_MasterReceiveData
 * @brief		Send data on the I2C peripheral
 * @inputs
 * 		pI2CHandle:		I2Cx handle
 * 		pRxbuffer:		Data to be received
 * 		Len:			Length of data
 * 		SlaveAddrs:		Address of slave
 * 		Sr:				Disable or enable repeated start
 *
 * @return		None
 *************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddrs, uint8_t Sr)
{

	//1. Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddrs);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR )
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP_BIT_POS);

		//read data in to buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR )
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP_BIT_POS);

			}

			//read the data from data register in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxbuffer++;

		}
	}

}

/*************************************************************
 * @name		I2C_MasterSendDataIT
 * @brief		Send data on the I2C peripheral with interrupt
 * @inputs
 * 		pI2CHandle:		I2Cx handle
 * 		pRxbuffer:		Data to be received
 * 		Len:			Length of data
 * 		SlaveAddrs:		Address of slave
 * 		Sr:				Disable or enable repeated start
 *
 * @return		State
 *************************************************************/
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);

		//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN_BIT_POS);

		//Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN_BIT_POS);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN_BIT_POS);
	}

	return busystate;
}

/*************************************************************
 * @name		I2C_MasterReceiveDataIT
 * @brief		Receive data on the I2C peripheral with interrupt
 * @inputs
 * 		pI2CHandle:		I2Cx handle
 * 		pRxbuffer:		Data to be received
 * 		Len:			Length of data
 * 		SlaveAddrs:		Address of slave
 * 		Sr:				Disable or enable repeated start
 *
 * @return		State
 *************************************************************/
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddrs, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxbuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddrs;
		pI2CHandle->Sr = Sr;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);

		//Enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN_BIT_POS);

		//Enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN_BIT_POS);

		//Enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN_BIT_POS);
	}

	return busystate;
}

/*************************************************************
 * @name		I2C_EV_IQRHandling
 * @brief		Handle I2C event interrupts
 * @inputs
 * 		pI2CHandle:			I2C Handle
 *
 * @return		None
 *************************************************************/
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint8_t temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN_BIT_POS);
	uint8_t temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN_BIT_POS);

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	uint8_t temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB_BIT_POS);

	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR_BIT_POS);

	if(temp1 && temp3)
	{
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

	}

	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF_BIT_POS);

	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1 & (1<< I2C_SR1_TXE_BIT_POS))
			{
				if(pI2CHandle->TxLen == 0)
				{
					//Generate stop condition if repeated start was disabled
					if(pI2CHandle->Sr == DISABLE)
					{
						pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP_BIT_POS);
					}

					//Reset all members in the handle function
					I2C_CloseSendData(pI2CHandle);

					//notify the application about send complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;		//Nothing to do here
		}
	}

	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF_BIT_POS);

	if(temp1 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE_BIT_POS))
		{
			//Clear STOPF by reading SR1 and then writing to CR1
			//We already read SR1 with the if statement above
			pI2CHandle->pI2Cx->CR1 |= 0x00;				//We don't want to change the vale of CR1

			//notify the application
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
		}

	}

	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE_BIT_POS);

	if(temp1 && temp2 && temp3)
	{
		//check if the device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL_BIT_POS))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					pI2CHandle->pI2Cx->DR = *pI2CHandle->pTxBuffer;
					pI2CHandle->pTxBuffer++;						//increment the buffer address
					pI2CHandle->TxLen--;							//decrement the length of data

				}
			}
		}

	}

	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE_BIT_POS);

	if(temp1 && temp2 && temp3)
	{
		//if device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL_BIT_POS))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				if(pI2CHandle->RxSize == 1)
				{
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
				}

				if(pI2CHandle->RxSize > 1)
				{
					if(pI2CHandle->RxLen == 2)
					{
						I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
					}

					//Read data from DR
					*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
					pI2CHandle->RxLen--;
					pI2CHandle->pRxBuffer++;
				}

				if(pI2CHandle->RxSize == 0)
				{
					//Generate stop condition if repeated start was disabled
					if(pI2CHandle->Sr == DISABLE)
					{
						pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP_BIT_POS);
					}

					I2C_CloseReceiveData(pI2CHandle);

					//notify the application
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
				}
			}
		}
	}
}

/*************************************************************
 * @name		I2C_ER_IRQHandling
 * @brief		Handle I2C error interrupts
 * @inputs
 * 		pI2CHandle:			I2C Handle
 *
 * @return		None
 *************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN_BIT_POS);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR_BIT_POS);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR_BIT_POS);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error***********************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO_BIT_POS);
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO_BIT_POS);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error***************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF_BIT_POS);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF_BIT_POS);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error***********************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR_BIT_POS);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR_BIT_POS);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error*******************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT_BIT_POS);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT_BIT_POS);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}
}

/*************************************************************
 * @name		I2C_ExecuteAddressPhase
 * @brief
 * @inputs
 * 		pI2Cx:			I2Cx base address
 * 		SlaveAddrs:		Address of slave
 *
 * @return		None
 *************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;		//To make space for the read/write bit
	SlaveAddr &= ~(1);					//Clear the zeroth bit/
	pI2Cx->DR = SlaveAddr;
}

/*************************************************************
 * @name		I2C_ExecuteAddressPhaseRead
 * @brief
 * @inputs
 * 		pI2Cx:			I2Cx base address
 * 		SlaveAddrs:		Address of slave
 *
 * @return		None
 *************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

/*************************************************************
 * @name		I23C_ClearADDRFlag
 * @brief		Clear Address flag
 * @inputs
 * 		pI2Cx:			I2Cx base address
 *
 * @return		None
 *************************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL_BIT_POS))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize  == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);

				//clear the ADDR flag ( read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			//clear the ADDR flag ( read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		//device is in slave mode
		//clear the ADDR flag ( read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}

}

/*************************************************************
 * @name		I2C_ManageAcking
 * @brief		Clear Address flag
 * @inputs
 * 		pI2Cx:			I2Cx base address
 * 		EnorDi:			ENABLE or DISABLE
 *
 * @return		None
 *************************************************************/
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK_BIT_POS);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK_BIT_POS);
	}
}

/*************************************************************
 * @name		I2C_CloseReceiveData
 * @brief		Disable interrupts when receiving data
 * @inputs
 * 		pI2CHandle:			I2C Handle
 *
 * @return		None
 *************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN_BIT_POS);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN_BIT_POS);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}

}

/*************************************************************
 * @name		I2C_CloseSendData
 * @brief		Disable interrupts when sending data
 * @inputs
 * 		pI2CHandle:			I2C Handle
 *
 * @return		None
 *************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN_BIT_POS);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN_BIT_POS);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*************************************************************
 * @name		I2C_IRQInterruptConfig
 * @brief		Configure interrupts
 * @inputs
 * 		IRQNumber:		Interrupt request number
 * 		EnorDi:			ENABLE or DISABLE
 *
 * @return		None
 *************************************************************/
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
