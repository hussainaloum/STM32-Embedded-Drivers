/*
 * 006spi_tx_testing.c
 *
 *  Created on: Nov 17, 2021
 *      Author: ha
 */

#include "stm32f439zi_spi_driver.h"
#include "stm32f439zi_gpio_driver.h"
#include <string.h>						//for strlen()

/*************************************************************
 * @name		SPI_GPIO_Init
 * @brief		Initialize GPIOx port to SPI alternate function
 * @inputs
 * 		GPIO_BASEADDR:		GPIOx port base address
 *
 * @return		None
 *************************************************************/
void SPI_GPIO_Init(GPIO_RegDef_t *GPIOx_BASEADDR);

int main(void)
{

	/*
	 * Connections
	 * SCLK	-->	PB10
	 * MOSI	--> PB15
	 * SS	--> PB9
	 * MISO	--> Not used here since slave is not sending data
	 *
	 * This program sends data "Hello World" on the SPI 2 bus
	 * To do that follow these steps
	 * 1- Pick SPI pins (MOSI, MISO, SCLK, SS) and Initialize the pins
	 * 2- Initialize SPI
	 * */

	//1- Initialize the GPIO to SPI alternate function
	SPI_GPIO_Init(GPIOB);

	//2- Initialize SPI
	SPI_Handle_t SPI2_Handle;

	SPI2_Handle.pSPIx						= 	SPI2;
	SPI2_Handle.SPIConfig.SPI_DeviceMode	=	SPI_DEVICE_MODE_MASTER;
	SPI2_Handle.SPIConfig.SPI_BusConfig		=	SPI_BUS_CONFIG_FD;
	SPI2_Handle.SPIConfig.SPI_SclkSpeed		=	SPI_SCLK_SPEED_DIV2;		//Generate SCLK of 8MHz
	SPI2_Handle.SPIConfig.SPI_DFF			=	SPI_DFF_8BITS;
	SPI2_Handle.SPIConfig.SPI_CPOL			=	SPI_CPOL_LOW;
	SPI2_Handle.SPIConfig.SPI_CPHA			=	SPI_CPHA_LOW;
	SPI2_Handle.SPIConfig.SPI_SSM			=	SPI_SSM_DI;					//Disable software SS management

	SPI_Init(&SPI2_Handle);


	//Set SSOE bit since the ST chip is operating in Master mode
	SPI_SSOEConfig(SPI2,ENABLE);

	//Enable the SPI2 peripheral (after all the configurations are done)
	SPI_PeripheralControl(SPI2, ENABLE);

	char user_data[] = "Hello World";

	//Send the data here
	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	//Disable SPI peripheral after the data is sent
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

}


void SPI_GPIO_Init(GPIO_RegDef_t *GPIOx_BASEADDR)
{
	GPIO_Handle_t GPIOx;

	GPIOx.pGPIOx								=	GPIOx_BASEADDR;
	GPIOx.GPIO_PinConfig.GPIO_PinMode			=	GPIO_MODE_ALT_FN;
	GPIOx.GPIO_PinConfig.GPIO_PinAltFunMode		=	5;
	GPIOx.GPIO_PinConfig.GPIO_PinOPType			=	GPIO_OP_TYPE_PP;
	GPIOx.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_NO_PUPD;
	GPIOx.GPIO_PinConfig.GPIO_PinSpeed			=	GPIO_SPEED_FAST;

	//SCLK
	GPIOx.GPIO_PinConfig.GPIO_PinNumber			=	10;
	GPIO_Init(&GPIOx);

	//SS
	GPIOx.GPIO_PinConfig.GPIO_PinNumber			=	9;
	GPIO_Init(&GPIOx);

	//MOSI
	GPIOx.GPIO_PinConfig.GPIO_PinNumber			=	15;
	GPIO_Init(&GPIOx);
}
