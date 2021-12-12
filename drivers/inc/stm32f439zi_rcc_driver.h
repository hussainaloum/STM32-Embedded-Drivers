/*
 * stm32f439zi_rcc_driver.h
 *
 *  Created on: Dec 10, 2021
 *      Author: ha
 */

#ifndef INC_STM32F439ZI_RCC_DRIVER_H_
#define INC_STM32F439ZI_RCC_DRIVER_H_

#include "stm32f439zi.h"

/************************************************
 	 	 	 	 APIs supported RCC
************************************************/

/*
 * Peripheral clock
 * */
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

/*
 * Other Peripheral control APIs
 * */
uint32_t  RCC_GetPLLOutputClock();

#endif /* INC_STM32F439ZI_RCC_DRIVER_H_ */
