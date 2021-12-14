#ifndef INC_STM32F439ZI
#define INC_STM32F439ZI

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>			//To use NULL

#define __vo 	volatile
#define __weak	__attribute__((weak))

/******************** Processor specific details ************************/
/*
 * ARM Cortex M4 processor NVIC ISERx register addresses	(Can be found in the Cortex M4 User Guide)
 * */
#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )

/*
 * ARM Cortex M4 processor NVIC ICERx register addresses
 * */
#define NVIC_ICER0			((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2			((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex M4 processor Priority register Address calculation
 * */
#define NVIC_PR_BASEADDR	((__vo uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED	4

/* Base addresses of Flash (main memory) and SRAM memories*/
#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x20001C00U
#define ROM						0x1FFF0000U			//ROM is System memory
#define SRAM					SRAM1_BASEADDR

/* Base addresses of AHPx and APBx Bus Peripherals */

#define	PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/* Base addresses of peripherals hanging on AHB1 bus */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/* Base addresses of peripherals hanging on APB1 bus */
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR			(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR			(APB1PERIPH_BASEADDR + 0x7C00)

/* Base addresses of peripherals hanging on APB2 bus */
#define	EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)


/**********************************Peripheral Register Definition Structures******************************/
typedef struct
{
	__vo uint32_t	MODER;			//GPIO port mode register
	__vo uint32_t	OTYPER;			//GPIO port output type register
	__vo uint32_t	OSPEEDR;		//GPIO port output speed register
	__vo uint32_t	PUPDR;			//GPIO port pull-up/pull-down register
	__vo uint32_t	IDR;			//GPIO port input data register
	__vo uint32_t	ODR;			//GPIO port output data register
	__vo uint32_t	BSRR;			//GPIO port bit set/reset register
	__vo uint32_t	LCKR;			//GPIO port configuration lock register
	__vo uint32_t	AFR[2];			//GPIO alternate function low/high registers
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t	RCC_CR;			//offset of 0x00
	__vo uint32_t	RCC_PLLCFGR;	//offset of 0x04
	__vo uint32_t	RCC_CFGR;		//offset of 0x08
	__vo uint32_t	RCC_CIR;		//offset of 0x0C
	__vo uint32_t	RCC_AHB1RSTR;	//offset of 0x10
	__vo uint32_t	RCC_AHB2RSTR;	//offset of 0x14
	__vo uint32_t	RCC_AHB3RSTR;	//offset of 0x18
	     uint32_t	Reserved0;		//offset of 0x1C
	__vo uint32_t	RCC_APB1RSTR;	//offset of 0x20
	__vo uint32_t	RCC_APB2RSTR;	//offset of 0x24
		 uint32_t	Reserved1[2];	//offset of 0x28
	__vo uint32_t	RCC_AHB1ENR;	//offset of 0x30
	__vo uint32_t	RCC_AHB2ENR;	//offset of 0x34
	__vo uint32_t	RCC_AHB3ENR;	//offset of 0x38
		 uint32_t	Reserved2;		//offset of 0x3C
	__vo uint32_t	RCC_APB1ENR;	//offset of 0x40
	__vo uint32_t	RCC_APB2ENR;	//offset of 0x44
		 uint32_t	Reserved3[2];	//offset of 0x48
	__vo uint32_t	RCC_AHB1LPENR;	//offset of 0x50
	__vo uint32_t	RCC_AHB2LPENR;	//offset of 0x54
	__vo uint32_t	RCC_AHB3LPENR;	//offset of 0x58
		 uint32_t	Reserved4;		//offset of 0x5C
	__vo uint32_t	RCC_APB1LPENR;	//offset of 0x60
	__vo uint32_t	RCC_APB2LPENR;	//offset of 0x64
		 uint32_t	Reserved5[2];	//offset of 0x68
	__vo uint32_t	RCC_BDCR;		//offset of 0x70
	__vo uint32_t	RCC_CSR;		//offset of 0x74
		 uint32_t	Reserved6[2];	//offset of 0x78
	__vo uint32_t	RCC_SSCGR;		//offset of 0x80
	__vo uint32_t	RCC_PLLI2SCFGR;	//offset of 0x84
	__vo uint32_t	RCC_PLLSAICFGR;	//offset of 0x88
	__vo uint32_t	RCC_DCKCFGR;	//offset of 0x8C

}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t	IMR;			//offset of 0x00
	__vo uint32_t	EMR;			//offset of 0x04
	__vo uint32_t	RTSR;			//offset of 0x08
	__vo uint32_t	FTSR;			//offset of 0x0C
	__vo uint32_t	SWIER;			//offset of 0x10
	__vo uint32_t	PR;				//offset of 0x14

}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t	SYSCFG_MEMRMP;		//offset of 0x00
	__vo uint32_t	SYSCFG_PMC;			//offset of 0x04
	__vo uint32_t	SYSCFG_EXTICR[4];	//offset of 0x08 - 0x14
	__vo uint32_t	SYSCFG_CMPCR;		//offset of 0x20
}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t		SPI_CR1;
	__vo uint32_t		SPI_CR2;
	__vo uint32_t		SPI_SR;
	__vo uint32_t		SPI_DR;
	__vo uint32_t		SPI_CRCPR;
	__vo uint32_t		SPI_RXCRCR;
	__vo uint32_t		SPI_TXCRCR;
	__vo uint32_t		SPI_I2SCFGR;
	__vo uint32_t		SPI_I2SPR;
}SPI_RegDef_t;

typedef struct
{
	__vo uint32_t		CR1;
	__vo uint32_t		CR2;
	__vo uint32_t		OAR1;
	__vo uint32_t		OAR2;
	__vo uint32_t		DR;
	__vo uint32_t		SR1;
	__vo uint32_t		SR2;
	__vo uint32_t		CCR;
	__vo uint32_t		TRISE;
	__vo uint32_t		FLTR;
}I2C_RegDef_t;

typedef struct
{
	__vo uint32_t		SR;
	__vo uint32_t		DR;
	__vo uint32_t		BRR;
	__vo uint32_t		CR1;
	__vo uint32_t		CR2;
	__vo uint32_t		CR3;
	__vo uint32_t		GTPR;
}USART_RegDef_t;

/* Peripheral Definitions */

#define GPIOA					((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI					((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define GPIOJ					((GPIO_RegDef_t *)GPIOJ_BASEADDR)
#define GPIOK					((GPIO_RegDef_t *)GPIOK_BASEADDR)

#define RCC						((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t *)SPI4_BASEADDR)

#define I2C1					((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1					((USART_RegDef_t *)USART1_BASEADDR)
#define USART2					((USART_RegDef_t *)USART2_BASEADDR)
#define USART3					((USART_RegDef_t *)USART3_BASEADDR)
#define USART6					((USART_RegDef_t *)USART6_BASEADDR)
#define UART4					((USART_RegDef_t *)UART4_BASEADDR)		//(USART_RegDef_t *) is used for UART also
#define UART5					((USART_RegDef_t *)UART5_BASEADDR)		//(USART_RegDef_t *) is used for UART also
#define UART7					((USART_RegDef_t *)UART7_BASEADDR)		//(USART_RegDef_t *) is used for UART also
#define UART8					((USART_RegDef_t *)UART8_BASEADDR)		//(USART_RegDef_t *) is used for UART also

/* Clock Enable Macros for GPIOx peripherals */

#define GPIOA_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 0))
#define GPIOB_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 1))
#define GPIOC_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 2))
#define GPIOD_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 3))
#define GPIOE_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 4))
#define GPIOF_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 5))
#define GPIOG_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 6))
#define GPIOH_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 7))
#define GPIOI_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 8))
#define GPIOJ_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 9))
#define GPIOK_PCLK_EN()		(RCC->RCC_AHB1ENR |= (0x01 << 10))

/* Clock Enable Macros for I2Cx peripherals */

#define I2C1_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 21))
#define I2C2_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 22))
#define I2C3_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 23))

/* Clock Enable Macros for SPIx peripherals */

#define SPI2_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 14))
#define SPI3_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 15))

#define SPI1_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x01 << 12))
#define SPI4_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x01 << 13))
#define SPI5_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x01 << 20))
#define SPI6_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x01 << 21))

/* Clock Enable Macros for USARTx peripherals */

#define USART2_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 17))
#define USART3_PCLK_EN()		(RCC->RCC_APB1ENR |= (0x01 << 18))
#define UART4_PCLK_EN()			(RCC->RCC_APB1ENR |= (0x01 << 19))
#define UART5_PCLK_EN()			(RCC->RCC_APB1ENR |= (0x01 << 20))
#define UART7_PCLK_EN()			(RCC->RCC_APB1ENR |= (0x01 << 30))
#define UART8_PCLK_EN()			(RCC->RCC_APB1ENR |= (0x01 << 31))

#define USART1_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x01 << 4))
#define USART6_PCLK_EN()		(RCC->RCC_APB2ENR |= (0x01 << 5))

/* Clock Enable Macros for SYSCFG peripherals */

#define SYSCFG_PCLK_EN()			(RCC->RCC_APB2ENR |= (0x01 << 14))

/* Macros to reset GPIOx ports */

#define GPIOA_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 0)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 0)); }while(0)	//do{}while() is used to combine to commands in one line
#define GPIOB_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 1)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 2)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 3)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 4)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 4)); }while(0)
#define GPIOF_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 5)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 5)); }while(0)
#define GPIOG_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 6)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 6)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 7)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 7)); }while(0)
#define GPIOI_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 8)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 8)); }while(0)
#define GPIOJ_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 9)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 9)); }while(0)
#define GPIOK_REG_RESET()		do{ (RCC->RCC_AHB1RSTR |= (0x01 << 10)); (RCC->RCC_AHB1RSTR &= ~(0x01 << 10)); }while(0)

/* Macros to reset SPIx peripheral */
#define SPI1_RST()				do{ (RCC->RCC_APB2RSTR |= (0x01 << 12)); (RCC->RCC_APB2RSTR &= ~(0x01 << 12)); }while(0)
#define SPI2_RST()				do{ (RCC->RCC_APB1RSTR |= (0x01 << 14)); (RCC->RCC_APB1RSTR &= ~(0x01 << 14)); }while(0)
#define SPI3_RST()				do{ (RCC->RCC_APB1RSTR |= (0x01 << 15)); (RCC->RCC_APB1RSTR &= ~(0x01 << 15)); }while(0)
#define SPI4_RST()				do{ (RCC->RCC_APB2RSTR |= (0x01 << 13)); (RCC->RCC_APB2RSTR &= ~(0x01 << 13)); }while(0)

/* Macros to reset I2Cx peripheral */
#define I2C1_RST()				do{ (RCC->RCC_APB1RSTR |= (0x01 << 21)); (RCC->RCC_APB1RSTR &= ~(0x01 << 21)); }while(0)
#define I2C2_RST()				do{ (RCC->RCC_APB1RSTR |= (0x01 << 22)); (RCC->RCC_APB1RSTR &= ~(0x01 << 22)); }while(0)
#define I2C3_RST()				do{ (RCC->RCC_APB1RSTR |= (0x01 << 23)); (RCC->RCC_APB1RSTR &= ~(0x01 << 23)); }while(0)

/* Return port code for a given GPIOx base address */

#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :\
										(x == GPIOJ) ? 9 :\
										(x == GPIOK) ? 10 :0	)	//if x equals GPIOA then make it equal to 0 or ..

/* Clock Disable Macros for GPIOx peripherals */

#define GPIOA_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 0))
#define GPIOB_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 1))
#define GPIOC_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 2))
#define GPIOD_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 3))
#define GPIOE_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 4))
#define GPIOF_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 5))
#define GPIOG_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 6))
#define GPIOH_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 7))
#define GPIOI_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 8))
#define GPIOJ_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 9))
#define GPIOK_PCLK_DIS()		(RCC->RCC_AHB1ENR &= ~(0x01 << 10))

/* Clock Disable Macros for I2Cx peripherals */

#define I2C1_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 21))
#define I2C2_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 22))
#define I2C3_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 23))

/* Clock Disable Macros for SPIx peripherals */

#define SPI2_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 14))
#define SPI3_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 15))

#define SPI1_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 12))
#define SPI4_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 13))
#define SPI5_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 20))
#define SPI6_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 21))

/* Clock Disable Macros for USARTx peripherals */

#define USART2_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 17))
#define USART3_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 18))
#define UART4_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 19))
#define UART5_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 20))
#define UART7_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 30))
#define UART8_PCLK_DIS()		(RCC->RCC_APB1ENR &= ~(0x01 << 31))

#define USART1_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 4))
#define USART6_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 5))

/* Clock Disable Macros for SYSCFG peripherals */

#define SYSCFG_PCLK_DIS()		(RCC->RCC_APB2ENR &= ~(0x01 << 14))

/* IRQ (interrupt request) Numbers of your MCU */
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

/* Macros for all the possible priority levels */
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI15			15
#define NVIC_IRQ_PRI13			13


/********************* Bit position definitions of SPI Peripheral *****************************/

/* Bit position definitions of SPI_CR1 */
#define SPI_CR1_CPHA_BIT_POS		0
#define SPI_CR1_CPOL_BIT_POS		1
#define SPI_CR1_MSTR_BIT_POS		2
#define SPI_CR1_BR_BIT_POS			3
#define SPI_CR1_SPE_BIT_POS			6
#define SPI_CR1_LSBFIRST_BIT_POS	7
#define SPI_CR1_SSI_BIT_POS			8
#define SPI_CR1_SSM_BIT_POS			9
#define SPI_CR1_RXONLY_BIT_POS		10
#define SPI_CR1_DFF_BIT_POS			11
#define SPI_CR1_CRCNEXT_BIT_POS		12
#define SPI_CR1_CRCEN_BIT_POS		13
#define SPI_CR1_BIDIOE_BIT_POS		14
#define SPI_CR1_BIDIMODE_BIT_POS	15

/* Bit position definitions of SPI_CR1 */
#define SPI_CR2_RXDMAEN_BIT_POS		0
#define SPI_CR2_TXDMAEN_BIT_POS		1
#define SPI_CR2_SSOE_BIT_POS		2
#define SPI_CR2_FRF_BIT_POS			4
#define SPI_CR2_ERRIE_BIT_POS		5
#define SPI_CR2_RXNEIE_BIT_POS		6
#define SPI_CR2_TXEIE_BIT_POS		7

/* Bit position definitions of SPI_SR */
#define SPI_SR_RXNE_BIT_POS			0
#define SPI_SR_TXE_BIT_POS			1
#define SPI_SR_CHSIDE_BIT_POS		2
#define SPI_SR_UDR_BIT_POS			3
#define SPI_SR_CRCERR_BIT_POS		4
#define SPI_SR_MODF_BIT_POS			5
#define SPI_SR_OVR_BIT_POS			6
#define SPI_SR_BSY_BIT_POS			7
#define SPI_SR_FRE_BIT_POS			8

/********************* Bit position definitions of I2C Peripheral *****************************/

/* Bit position definitions of I2C_CR1 */
#define I2C_CR1_PE_BIT_POS			0
#define I2C_CR1_SMBUS_BIT_POS		1
#define I2C_CR1_SMBTYPE_BIT_POS		3
#define I2C_CR1_ENARP_BIT_POS		4
#define I2C_CR1_ENPEC_BIT_POS		5
#define I2C_CR1_ENGC_BIT_POS		6
#define I2C_CR1_NOSTRETCH_BIT_POS	7
#define I2C_CR1_START_BIT_POS		8
#define I2C_CR1_STOP_BIT_POS		9
#define I2C_CR1_ACK_BIT_POS			10
#define I2C_CR1_POS_BIT_POS			11
#define I2C_CR1_PEC_BIT_POS			12
#define I2C_CR1_ALERT_BIT_POS		13
#define I2C_CR1_SWRST_BIT_POS		15

/* Bit position definitions of I2C_CR2 */
#define I2C_CR2_FREQ_BIT_POS		0
#define I2C_CR2_ITERREN_BIT_POS		8
#define I2C_CR2_ITEVTEN_BIT_POS		9
#define I2C_CR2_ITBUFEN_BIT_POS		10
#define I2C_CR2_DMAEN_BIT_POS		11
#define I2C_CR2_LAST_BIT_POS		12

/* Bit position definitions of I2C_SR1 */
#define I2C_SR1_SB_BIT_POS			0
#define I2C_SR1_ADDR_BIT_POS		1
#define I2C_SR1_BTF_BIT_POS			2
#define I2C_SR1_ADD10_BIT_POS		3
#define I2C_SR1_STOPF_BIT_POS		4
#define I2C_SR1_RXNE_BIT_POS		6
#define I2C_SR1_TXNE_BIT_POS		7
#define I2C_SR1_BERR_BIT_POS		8
#define I2C_SR1_ARLO_BIT_POS		9
#define I2C_SR1_AF_BIT_POS			10
#define I2C_SR1_OVR_BIT_POS			11
#define I2C_SR1_PECERR_BIT_POS		12
#define I2C_SR1_TIMEOUT_BIT_POS		14
#define I2C_SR1_SMBALERT_BIT_POS	15

/* Bit position definitions of I2C_SR2 */
#define I2C_SR2_MSL_BIT_POS			0
#define I2C_SR2_BUSY_BIT_POS		1
#define I2C_SR2_TRA_BIT_POS			2
#define I2C_SR2_GENCALL_BIT_POS		4
#define I2C_SR2_SMBDEFAULT_BIT_POS	5
#define I2C_SR2_SMBHOST_BIT_POS		6
#define I2C_SR2_DUALF_BIT_POS		7
#define I2C_SR2_PEC_BIT_POS			8

/* Bit position definitions of I2C_CCR */
#define I2C_CCR_CCR_BIT_POS			0
#define I2C_CCR_DUTY_BIT_POS		14
#define I2C_CCR_FS_BIT_POS			15

/* Bit position definitions of I2C_TRISE */
#define I2C_TRISE_TRISE_BIT_POS		0

/********************* Bit position definitions of USART/UART Peripheral *****************************/

/* Bit position definitions of USART/UART_SR */
#define USART_SR_PE_BIT_POS			0
#define USART_SR_FE_BIT_POS			1
#define USART_SR_NF_BIT_POS			2
#define USART_SR_ORE_BIT_POS		3
#define USART_SR_IDLE_BIT_POS		4
#define USART_SR_RXNE_BIT_POS		5
#define USART_SR_TC_BIT_POS			6
#define USART_SR_TXE_BIT_POS		7
#define USART_SR_LBD_BIT_POS		8
#define USART_SR_CTS_BIT_POS		9

/* Bit position definitions of USART/UART_CR1 */
#define USART_CR1_SBK_BIT_POS		0
#define USART_CR1_RWU_BIT_POS		1
#define USART_CR1_RE_BIT_POS		2
#define USART_CR1_TE_BIT_POS		3
#define USART_CR1_IDLEIE_BIT_POS	4
#define USART_CR1_RXNEIE_BIT_POS	5
#define USART_CR1_TCIE_BIT_POS		6
#define USART_CR1_TXEIE_BIT_POS		7
#define USART_CR1_PEIE_BIT_POS		8
#define USART_CR1_PS_BIT_POS		9
#define USART_CR1_PCE_BIT_POS		10
#define USART_CR1_WAKE_BIT_POS		11
#define USART_CR1_M_BIT_POS			12
#define USART_CR1_UE_BIT_POS		13
#define USART_CR1_OVER8_BIT_POS		15

/* Bit position definitions of USART/UART_CR2 */
#define USART_CR2_ADD3_BIT_POS		3
#define USART_CR2_LBDL_BIT_POS		5
#define USART_CR2_LBDIE_BIT_POS		6
#define USART_CR2_LBCL_BIT_POS		8
#define USART_CR2_CPHA_BIT_POS		9
#define USART_CR2_CPOL_BIT_POS		10
#define USART_CR2_CLKEN_BIT_POS		11
#define USART_CR2_STOP_BIT_POS		13
#define USART_CR2_LINEN_BIT_POS		14

/* Bit position definitions of USART/UART_CR3 */
#define USART_CR3_EIE_BIT_POS		0
#define USART_CR3_IREN_BIT_POS		1
#define USART_CR3_IRLP_BIT_POS		2
#define USART_CR3_HDSEL_BIT_POS		3
#define USART_CR3_NACK_BIT_POS		4
#define USART_CR3_SCEN_BIT_POS		5
#define USART_CR3_DMAR_BIT_POS		6
#define USART_CR3_DMAT_BIT_POS		7
#define USART_CR3_RTSE_BIT_POS		8
#define USART_CR3_CTSE_BIT_POS		9
#define USART_CR3_CTSIE_BIT_POS		10
#define USART_CR3_ONEBIT_BIT_POS	11

/******************************* Some generic macros ***********************************/
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET

#endif	/* INC_STM32F439ZI_H_ */
