/*
 * 007SPIExercise.c
 *
 *  Created on: Sep 14, 2024
 *      Author: Vishal Turaga
 *
 *  Exercise: STM32 Discovery board (master) receives the message from the Arduino board (Slave) over SPI
 *
 *  1. User enters the message using Arduino serial monitor terminal
 *  2. Arduino board notifies the STM32 board about message availabilty
 *  3. STM32 device reads and prints the message
 */



#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"
#include <string.h>
#include <stdio.h>

#define MAX_LEN 500

volatile uint8_t rcvStop = 0; //indicates when to stop

volatile uint8_t dataAvailable = 0; // this flag will be set in the interrupt handler of the arduino interrupt GPIO

SPI_Handle_t SPI2handle;

char RcvBuff[MAX_LEN];

volatile char ReadByte;




void GPIO_Inits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}

void SPI_Inits(void)
{
	SPI_Handle_t SPI_2;
	SPI_2.pSPIx = SPI2;
	SPI_2.SPIConfig.BusConfig = SPI_BUSCONFIG_FULLDUPLEX;
	SPI_2.SPIConfig.DeviceMode = SPI_DEVICEMODE_MASTER;
	SPI_2.SPIConfig.CPHA = SPI_CPHA_TRAILING_EDGE;
	SPI_2.SPIConfig.CPOL = SPI_CPOL_LOW_IDLE_STATE;
	SPI_2.SPIConfig.DFF = SPI_DFF_8BIT;
	SPI_2.SPIConfig.SclkSpeed = SPI_SCLKSPEED_FPCLK_32;
	SPI_2.SPIConfig.SSM = SPI_SSM_DI;

	SPI_Init(&SPI_2);
}

void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t SPI_IT_Pin;
	memset(&SPI_IT_Pin,0,sizeof(SPI_IT_Pin));
	SPI_IT_Pin.pGPIOx = GPIOD;
	SPI_IT_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	SPI_IT_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	SPI_IT_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_LOW;
	SPI_IT_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Init(&SPI_IT_Pin);

	GPIO_IRQITConfig(IRQ_EXTI9_5,ENABLE);
}

int main(void)
{
	uint8_t dummy = 0xff;

	GPIO_Inits();

	SPI_Inits();

	Slave_GPIO_InterruptPinInit();

	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_SPI2, ENABLE);

	while(1)
	{
		while(!dataAvailable); // wait till data available interrupt from transmitter device (slave)

		GPIO_IRQITConfig(IRQ_EXTI9_5, DISABLE);

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop)
		{
			// fetch the data from the SPI peripheral byte by byte in interrupt mode
			while(SPI_SendDataIT(&SPI2handle, &dummy, 1) == SPI_BUSY_IN_TX); // send a dummy byte before fetching data
			while(SPI_ReceiveDataIT(&SPI2handle, &ReadByte,1) == SPI_BUSY_IN_RX); //
		}

		// confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		// Disable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0;

		GPIO_IRQITConfig(IRQ_EXTI9_5, ENABLE);

	}

	return 0;
}

// slave data available interrupt handler
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_6);
	dataAvailable = 1;
}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	static uint32_t i = 0;
	// in the Rx complete Event, copy data in the rcv buffer. '\0' indicates end of message (rcvStop = 1)
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = ReadByte;
		if(ReadByte == '\0' || (i == MAX_LEN))
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i=0;
		}
	}
}
