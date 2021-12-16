/*
 * stm32f439zi_i2c_driver.c
 *
 *  Created on: Nov 19, 2021
 *      Author: ha
 */

#include "stm32f439zi_i2c_driver.h"
#include "stm32f439zi_rcc_driver.h"

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);


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
 *
 * @return		None
 *************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddrs)
{
	//1- Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START_BIT_POS);


	//2- Confirm start generation is completed by checking the SB flag in SR1
	//Not: until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3- Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddrs);

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
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP_BIT_POS);

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
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;		//To make space for the read/write bit
	SlaveAddr &= ~(1);					//Clear the zeroth bit/
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
