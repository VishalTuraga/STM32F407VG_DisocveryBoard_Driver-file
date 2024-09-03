/*
 * stm32f407xx.h
 *
 *  Created on: Aug 25, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#define __vo							volatile

/*
 * ARM Cortex Mx Processor Specific NVIC ISERx register details
 */
#define NVIC_ISER0						((__vo uint32_t*)0xe000e100)
#define NVIC_ISER1						((__vo uint32_t*)0xe000e104)
#define NVIC_ISER2						((__vo uint32_t*)0xe000e108)
#define NVIC_ISER3						((__vo uint32_t*)0xe000e10c)
#define NVIC_ISER4						((__vo uint32_t*)0xe000e110)
#define NVIC_ISER5						((__vo uint32_t*)0xe000e114)
#define NVIC_ISER6						((__vo uint32_t*)0xe000e118)
#define NVIC_ISER7						((__vo uint32_t*)0xe000e11c)

/*
 * ARM Cortex Mx Processor Specific NVIC ICERx register details
 */
#define NVIC_ICER0						((__vo uint32_t*)0xe000e180)
#define NVIC_ICER1						((__vo uint32_t*)0xe000e184)
#define NVIC_ICER2						((__vo uint32_t*)0xe000e188)
#define NVIC_ICER3						((__vo uint32_t*)0xe000e18c)
#define NVIC_ICER4						((__vo uint32_t*)0xe000e190)
#define NVIC_ICER5						((__vo uint32_t*)0xe000e194)
#define NVIC_ICER6						((__vo uint32_t*)0xe000e198)
#define NVIC_ICER7						((__vo uint32_t*)0xe000e19c)

/*
 * ARM Cortex Mx Processor Specific NVIC ISPRx register details
 */
#define NVIC_ISPR0						((__vo uint32_t*)0xe000e200)
#define NVIC_ISPR1						((__vo uint32_t*)0xe000e204)
#define NVIC_ISPR2						((__vo uint32_t*)0xe000e208)
#define NVIC_ISPR3						((__vo uint32_t*)0xe000e20c)
#define NVIC_ISPR4						((__vo uint32_t*)0xe000e210)
#define NVIC_ISPR5						((__vo uint32_t*)0xe000e214)
#define NVIC_ISPR6						((__vo uint32_t*)0xe000e218)
#define NVIC_ISPR7						((__vo uint32_t*)0xe000e21c)

/*
 * ARM Cortex Mx Processor Specific NVIC ICPRx register details
 */
#define NVIC_ICPR0						((__vo uint32_t*)0xe000e280)
#define NVIC_ICPR1						((__vo uint32_t*)0xe000e284)
#define NVIC_ICPR2						((__vo uint32_t*)0xe000e288)
#define NVIC_ICPR3						((__vo uint32_t*)0xe000e28c)
#define NVIC_ICPR4						((__vo uint32_t*)0xe000e290)
#define NVIC_ICPR5						((__vo uint32_t*)0xe000e294)
#define NVIC_ICPR6						((__vo uint32_t*)0xe000e298)
#define NVIC_ICPR7						((__vo uint32_t*)0xe000e29c)

/*
 * ARM Cortex Mx Processor Specific NVIC IABRx register details
 */
#define NVIC_IABR0						((__vo uint32_t*)0xe000e300)
#define NVIC_IABR1						((__vo uint32_t*)0xe000e304)
#define NVIC_IABR2						((__vo uint32_t*)0xe000e308)
#define NVIC_IABR3						((__vo uint32_t*)0xe000e30c)
#define NVIC_IABR4						((__vo uint32_t*)0xe000e310)
#define NVIC_IABR5						((__vo uint32_t*)0xe000e314)
#define NVIC_IABR6						((__vo uint32_t*)0xe000e318)
#define NVIC_IABR7						((__vo uint32_t*)0xe000e31c)

/*
 * ARM Cortex Mx Processor Specific NVIC IPRx register details
 */
#define NVIC_IPR_BASEADDR				((__vo uint32_t*)0xe000e400)

#define NO_PR_BITS_IMPLEMENTED			4

#define NVIC_STIR						((__vo uint32_t*)0xE000EF00)

/*
 * Define base addresses of Flash and SRAM memories
 */


#define FLASH_BASEADDR					0x08000000UL					/*Base address of the flash memory*/
#define SRAM_BASEADDR					0x20000000UL					/*Base address of the SRAM memory*/
#define SRAM_SIZE						((128)*(1024))					/*Base address of the flash memory*/
#define SRAM1_BASEADDR					SRAM_BASEADDR					/*Base address of the flash memory*/
#define SRAM2_BASEADDR					0x2001C000UL					/*Base address of the flash memory*/
#define ROM_BASEADDR					0x1FFF0000UL					/*Base address of the flash memory*/
#define ROM_SIZE						((ROM_BASEADDR) + (30 * 1024))	/*Base address of the flash memory*/

/*
 *	Base addresses of Bus domains
 */

#define AHB3_BASEADDR					0xA0000000UL					/*Base address of the AHB3 bus*/
#define AHB2_BASEADDR					0x50000000UL					/*Base address of the AHB2 bus*/
#define AHB1_BASEADDR					0x40020000UL					/*Base address of the AHB1 bus*/
#define APB2_BASEADDR					0x40010000UL					/*Base address of the APB2 bus*/
#define APB1_BASEADDR					0x40000000UL					/*Base address of the APB1 bus*/
#define PERIPH_BASEADDR					0x40000000UL					/*Base address from where peripherals start*/

/*
 *	Base addresses of AHB1 peripherals
 */
#define GPIOA_BASEADDR					0x40020000UL
#define GPIOB_BASEADDR					0x40020400UL
#define GPIOC_BASEADDR					0x40020800UL
#define GPIOD_BASEADDR					0x40020C00UL
#define GPIOE_BASEADDR					0x40021000UL
#define GPIOF_BASEADDR					0x40021400UL
#define GPIOG_BASEADDR					0x40021800UL
#define GPIOH_BASEADDR					0x40021C00UL
#define GPIOI_BASEADDR					0x40022000UL
#define GPIOJ_BASEADDR					0x40022400UL
#define GPIOK_BASEADDR					0x40022800UL
#define CRC_BASEADDR					0x40023000UL
#define RCC_BASEADDR					0x40023800UL
#define FLASHINTERFACE_BASEADDR			0x40023C00UL
#define BKPSRAM_BASEADDR				0x40024000UL
#define DMA1_BASEADDR					0x40026000UL
#define DMA2_BASEADDR					0x40026400UL
#define ETHERNET_MAC_BASEADDR			0x40028000UL
#define DMA2D_BASEADDR					0x4002B000UL
#define USB_OTG_HS_BASEADDR				0x40040000UL

/*
 *	Base addresses of APB1 peripherals
 */


#define TIM2_BASEADDR					((APB1_BASEADDR) + 0x0000)
#define TIM3_BASEADDR					((APB1_BASEADDR) + 0x0400)
#define TIM4_BASEADDR					((APB1_BASEADDR) + 0x0800)
#define TIM5_BASEADDR					((APB1_BASEADDR) + 0x0C00)
#define TIM6_BASEADDR					((APB1_BASEADDR) + 0x1000)
#define TIM7_BASEADDR					((APB1_BASEADDR) + 0x1400)
#define TIM12_BASEADDR					((APB1_BASEADDR) + 0x1800)
#define TIM13_BASEADDR					((APB1_BASEADDR) + 0x1C00)
#define TIM14_BASEADDR					((APB1_BASEADDR) + 0x2000)
#define RTC_BKP_BASEADDR				((APB1_BASEADDR) + 0x2800)
#define WWDG_BASEADDR					((APB1_BASEADDR) + 0x2c00)
#define IWDG_BASEADDR					((APB1_BASEADDR) + 0x3000)
#define I2S2ext_BASEADDR				((APB1_BASEADDR) + 0x3400)
#define SPI2_I2S2_BASEADDR				((APB1_BASEADDR) + 0x3800)
#define SPI3_I2S3_BASEADDR				((APB1_BASEADDR) + 0x3c00)
#define I2S3ext_BASEADDR				((APB1_BASEADDR) + 0x4000)
#define USART2_BASEADDR					((APB1_BASEADDR) + 0x4400)
#define USART3_BASEADDR					((APB1_BASEADDR) + 0x4800)
#define UART4_BASEADDR					((APB1_BASEADDR) + 0x4c00)
#define UART5_BASEADDR					((APB1_BASEADDR) + 0x5000)
#define I2C1_BASEADDR					((APB1_BASEADDR) + 0x5400)
#define I2C2_BASEADDR					((APB1_BASEADDR) + 0x5800)
#define I2C3_BASEADDR					((APB1_BASEADDR) + 0x5c00)
#define CAN1_BASEADDR					((APB1_BASEADDR) + 0x6400)
#define CAN2_BASEADDR					((APB1_BASEADDR) + 0x6800)
#define PWR_BASEADDR					((APB1_BASEADDR) + 0x7000)
#define DAC_BASEADDR					((APB1_BASEADDR) + 0x7400)
#define UART7_BASEADDR					((APB1_BASEADDR) + 0x7800)
#define UART8_BASEADDR					((APB1_BASEADDR) + 0x7C00)

/*
 *	Base addresses of APB1 peripherals
 */
#define TIM1_BASEADDR					((APB2_BASEADDR) + 0x0000)
#define TIM8_BASEADDR					((APB2_BASEADDR) + 0x0400)
#define USART1_BASEADDR					((APB2_BASEADDR) + 0x1000)
#define USART6_BASEADDR					((APB2_BASEADDR) + 0x1400)
#define ADC1_3_BASEADDR					((APB2_BASEADDR) + 0x2000)
#define SDIO_BASEADDR					((APB2_BASEADDR) + 0x2C00)
#define SPI1_BASEADDR					((APB2_BASEADDR) + 0x3000)
#define SPI4_BASEADDR					((APB2_BASEADDR) + 0x3400)
#define SYSCFG_BASEADDR					((APB2_BASEADDR) + 0x3800)
#define EXTI_BASEADDR					((APB2_BASEADDR) + 0x3C00)
#define TIM9_BASEADDR					((APB2_BASEADDR) + 0x4000)
#define TIM10_BASEADDR					((APB2_BASEADDR) + 0x4400)
#define TIM11_BASEADDR					((APB2_BASEADDR) + 0x4800)
#define SPI5_BASEADDR					((APB2_BASEADDR) + 0x5000)
#define SPI6_BASEADDR					((APB2_BASEADDR) + 0x5400)
#define SAI1_BASEADDR					((APB2_BASEADDR) + 0x5800)
#define LCDTFT_BASEADDR					((APB2_BASEADDR) + 0x6800)



 /******************************************peripheral register definition structures****************************************************/
/*
  * GPIO peripheral register structure
  */

typedef struct
{
	__vo uint32_t MODER;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t OTYPER;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t OSPEEDR;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t PUPDR;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t IDR;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t ODR;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t BSSR;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t LCKR;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t AFRL;				 /* Give a short description, address offset:0x04*/
	__vo uint32_t AFRH;				 /* Give a short description, address offset:0x04*/
}GPIO_RegDef_t;

/*
 * Peripheral definitions (Peripheral base address typecased to xxx_RegDef_t)
 */
#define GPIOA							((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF							((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG							((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI				 			((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 * RCC peripheral register structure
 */
typedef struct
{
	__vo uint32_t CR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t PLLCFGR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t CFGR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t CIR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB1RSTR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB2RSTR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB3RSTR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED0;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t APB1RSTR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t APB2RSTR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED1;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED2;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB1ENR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB2ENR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB3ENR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED3;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t APB1ENR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t APB2ENR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED4;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED5;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB1LPENR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB2LPENR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t AHB3LPENR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED6;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t APB1LPENR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t APB2LPENR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED7;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED8;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t BDCR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t CSR;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED9;					 /* Givea short Description, address offset: 0x00*/
	uint32_t 	  RESERVED10;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t SSCGR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t PLLI2SCFGR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t PLLSAICFGR;					 /* Givea short Description, address offset: 0x00*/
	__vo uint32_t DCKCFGR;					 /* Givea short Description, address offset: 0x00*/
}RCC_RegDef_t;

#define RCC								((RCC_RegDef_t*)RCC_BASEADDR)
/*
 * EXTI peripheral register structure
 */
typedef struct
{
	__vo uint32_t IMR;							/*Give a short Description, address offset: 0x00*/
	__vo uint32_t EMR;							/*Give a short Description, address offset: 0x00*/
	__vo uint32_t RTSR;							/*Give a short Description, address offset: 0x00*/
	__vo uint32_t FTSR;							/*Give a short Description, address offset: 0x00*/
	__vo uint32_t SWIER;						/*Give a short Description, address offset: 0x00*/
	__vo uint32_t PR;							/*Give a short Description, address offset: 0x00*/
}EXTI_RegDef_t;

#define EXTI							((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * SYSCFG peripheral register structure
 */
typedef struct
{
	__vo uint32_t MEMRMP;							/*Give a short Description, address offset: 0x00*/
	__vo uint32_t PMC;							/*Give a short Description, address offset: 0x00*/
	__vo uint32_t EXTICR[4];							/*Give a short Description, address offset: 0x00*/
		 uint32_t RESERVED1;
		 uint32_t RESERVED2;
	__vo uint32_t CMPCR;							/*Give a short Description, address offset: 0x00*/
}SYSCFG_RegDef_t;

#define SYSCFG							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Enable clock macros for GPIOx peripherals
 */
#define GPIOA_CLK_EN()					(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLK_EN()					(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLK_EN()					(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLK_EN()					(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLK_EN()					(RCC->AHB1ENR |= (1<<4))
#define GPIOF_CLK_EN()					(RCC->AHB1ENR |= (1<<5))
#define GPIOG_CLK_EN()					(RCC->AHB1ENR |= (1<<6))
#define GPIOH_CLK_EN()					(RCC->AHB1ENR |= (1<<7))
#define GPIOI_CLK_EN()					(RCC->AHB1ENR |= (1<<8))

/*
 * Enable clock macros for I2Cx peripherals
 */
#define I2C1_CLK_EN()					(RCC->APB1ENR |= (1<<21))
#define I2C2_CLK_EN()					(RCC->APB1ENR |= (1<<22))
#define I2C3_CLK_EN()					(RCC->APB1ENR |= (1<<23))

/*
 * Enable clock macros for SPIx peripherals
 */
#define SPI1_CLK_EN()					(RCC->APB2ENR |= (1<<12))
#define SPI2_CLK_EN()					(RCC->APB1ENR |= (1<<14))
#define SPI3_CLK_EN()					(RCC->APB1ENR |= (1<<15))
#define SPI4_CLK_EN()					(RCC->APB2ENR |= (1<<13))
#define SPI5_CLK_EN()					(RCC->APB2ENR |= (1<<20))
#define SPI6_CLK_EN()					(RCC->APB2ENR |= (1<<21))

/*
 * Enable clock macros for USARTx peripherals
 */
#define USART1_CLK_EN()					(RCC->APB2ENR |= (1<<4))
#define USART2_CLK_EN()					(RCC->APB1ENR |= (1<<17))
#define USART3_CLK_EN()					(RCC->APB1ENR |= (1<<18))
#define USART6_CLK_EN()					(RCC->APB2ENR |= (1<<5))


/*
 * Enable clock macros for UARTx peripherals
 */
#define UART4_CLK_EN()					(RCC->APB1ENR |= (1<<19))
#define UART5_CLK_EN()					(RCC->APB1ENR |= (1<<20))
#define UART7_CLK_EN()					(RCC->APB1ENR |= (1<<30))
#define UART8_CLK_EN()					(RCC->APB1ENR |= (1<<31))

/*
 * Enable clock macros for SYSCFG peripherals
 */
#define SYSCFG_CLK_EN()					(RCC->APB2ENR |= (1<<14))

/*
 * Disable clock macros for GPIOx peripherals
 */
#define GPIOA_CLK_DI()					(RCC->AHB1ENR |= (1<<0))
#define GPIOB_CLK_DI()					(RCC->AHB1ENR |= (1<<1))
#define GPIOC_CLK_DI()					(RCC->AHB1ENR |= (1<<2))
#define GPIOD_CLK_DI()					(RCC->AHB1ENR |= (1<<3))
#define GPIOE_CLK_DI()					(RCC->AHB1ENR |= (1<<4))
#define GPIOF_CLK_DI()					(RCC->AHB1ENR |= (1<<5))
#define GPIOG_CLK_DI()					(RCC->AHB1ENR |= (1<<6))
#define GPIOH_CLK_DI()					(RCC->AHB1ENR |= (1<<7))
#define GPIOI_CLK_DI()					(RCC->AHB1ENR |= (1<<8))

/*
 * Disable clock macros for I2Cx peripherals
 */
#define I2C1_CLK_DI()					(RCC->APB1ENR &= ~(1<<21))
#define I2C2_CLK_DI()					(RCC->APB1ENR &= ~(1<<22))
#define I2C3_CLK_DI()					(RCC->APB1ENR &= ~(1<<23))

/*
 * Disable clock macros for SPIx peripherals
 */
#define SPI1_CLK_DI()					(RCC->APB2ENR &= ~(1<<12))
#define SPI2_CLK_DI()					(RCC->APB1ENR &= ~(1<<14))
#define SPI3_CLK_DI()					(RCC->APB1ENR &= ~(1<<15))
#define SPI4_CLK_DI()					(RCC->APB2ENR &= ~(1<<13))
#define SPI5_CLK_DI()					(RCC->APB2ENR &= ~(1<<20))
#define SPI6_CLK_DI()					(RCC->APB2ENR &= ~(1<<21))

/*
 * Disable clock macros for USARTx peripherals
 */
#define USART1_CLK_DI()					(RCC->APB2ENR &= ~(1<<4))
#define USART2_CLK_DI()					(RCC->APB1ENR &= ~(1<<17))
#define USART3_CLK_DI()					(RCC->APB1ENR &= ~(1<<18))
#define USART6_CLK_DI()					(RCC->APB2ENR &= ~(1<<5))


/*
 * Disable clock macros for UARTx peripherals
 */
#define UART4_CLK_DI()					(RCC->APB1ENR &= ~(1<<19))
#define UART5_CLK_DI()					(RCC->APB1ENR &= ~(1<<20))
#define UART7_CLK_DI()					(RCC->APB1ENR &= ~(1<<30))
#define UART8_CLK_DI()					(RCC->APB1ENR &= ~(1<<31))

/*
 * Disable clock macros for SYSCFG peripherals
 */
#define SYSCFG_CLK_DI()					(RCC->APB2ENR &= ~(1<<14))

/*
 * IRQ Number Macros
 */
#define WWDG 0
#define PVD 1
#define TAMP_STAMP2
#define RTC_WKUP 3
#define FLASH4
#define RCC5
#define EXTI0 6
#define EXTI1 7
#define EXTI2 8
#define EXTI3 9
#define EXTI4 10
#define DMA1_STREAM0 11
#define DMA1_STREAM1 12
#define DMA1_STREAM2 13
#define DMA1_STREAM3 14
#define DMA1_STREAM4 15
#define DMA1_STREAM5 16
#define DMA1_STREAM6 17
#define ADC 18
#define CAN1_TX 19
#define CAN1_RX0 20
#define CAN1_RX1 21
#define CAN1_SCE 22
#define EXTI9_5 23
#define TIM1_BRK_TIM9 24
#define TIM1_UP_TIM10 25
#define TIM1_TRG_COM_TIM11 26
#define TIM1_CC 27
#define TIM2 28
#define TIM3 29
#define TIM4 30
#define I2C1_EV 31
#define I2C1_ER 32
#define I2C2_EV 33
#define I2C2_ER 34
#define SPI1 35
#define SPI2 36
#define USART1 37
#define USART2 38
#define USART3 39
#define EXTI15_10 40
#define RTC_ALARM 41
#define OTG_FS_WKUP 42
#define TIM8_BRK_TIM12 43
#define TIM8_BRK_TIM13 44
#define TIM8_TRG_COM_TIM14 45
#define TIM8_CC 46
#define DMA1_STREAM7 47
#define FSMC 48
#define SDIO 49
#define TIM5 50
#define SPI3 51
#define UART4 52
#define UART5 53
#define TIM6_DAC 54
#define TIM7 55
#define DMA2_STREAM0 56
#define DMA2_STREAM1 57
#define DMA2_STREAM2 58
#define DMA2_STREAM3 59
#define DMA2_STREAM4 60
#define ETH 61
#define ETH_WKUP 62
#define CAN2_TX 62
#define CAN2_RX0 64
#define CAN2_RX1 65
#define CAN2_SCE 66
#define OTG_FS 67
#define DMA2_STREAM5 68
#define DMA2_STREAM6 69
#define DMA2_STREAM7 70
#define USART6 71
#define I2C3_EV 72
#define I2C3_ER 73
#define OTG_HS_EP1_OUT 74
#define OTG_HS_EP1_IN 75
#define OTG_HS_WKUP 76
#define OTG_HS 77
#define DCMI 78
#define CRYP 79
#define HASH_RNG 80
#define FPU 81

/*
 * Generic Macos
 */
#define ENABLE							1
#define DISABLE							0
#define SET								ENABLE
#define RESET							DISABLE
#define GPIO_PIN_SET					SET
#define GPIO_PIN_RESET					RESET

/*
 * Generic functions
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :0)

#include "stm32f407xx_gpio.h"

#endif /* INC_STM32F407XX_H_ */
