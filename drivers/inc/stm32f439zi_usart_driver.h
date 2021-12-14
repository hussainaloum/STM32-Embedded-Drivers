/*
 * stm32f439zi_usart_driver.h
 *
 *  Created on: Dec 9, 2021
 *      Author: ha
 */

#ifndef INC_STM32F439ZI_USART_DRIVER_H_
#define INC_STM32F439ZI_USART_DRIVER_H_

#include "stm32f439zi.h"


/* This is a Configuration structure for a USART communication protocol */
typedef struct
{
	uint8_t		USART_Mode;
	uint32_t	USART_Baud;
	uint8_t		USART_NoOfStopBits;
	uint8_t 	USART_WordLength;
	uint8_t 	USART_ParityControl;
	uint8_t		USART_HWFlowControl;
}USART_Config_t;

/* Handle structure for USART communication protocol */
typedef struct
{
	USART_Config_t	USARTConfig;
	USART_RegDef_t	*pUSARTx;
}USART_Handle_t;

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX 	0
#define USART_MODE_ONLY_RX 	1
#define USART_MODE_TXRX  	2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					2400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE  0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

/*
 * USART flags
 */
#define USART_FLAG_TXE 			( 1 << USART_SR_TXE_BIT_POS)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE_BIT_POS)
#define USART_FLAG_TC 			( 1 << USART_SR_TC_BIT_POS)

/************************************************
 	 APIs supported by this USART/UART driver
************************************************/

/* Peripheral Clock Setup */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t ENorDi);

/* Init and De-Init */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/*
 * Data send and receive
 * */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length);
//uint8_t USART_SendDataWithIT(USART_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);			//For use with interrupt
//uint8_t USART_ReceiveDataWithIT(USART_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);		//For use with interrupt


/*
 * IRQ Configuration and ISR handling
 * */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral control APIs
 * */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);
bool USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t FlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t ENorDI);

#endif /* INC_STM32F439ZI_USART_DRIVER_H_ */
