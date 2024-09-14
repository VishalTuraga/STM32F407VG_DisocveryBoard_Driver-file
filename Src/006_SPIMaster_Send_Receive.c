/*
 * 005_STMMaster_ArduinoSlave.c
 *
 *  Created on: Sep 5, 2024
 *      Author: ASUS
 */


#include <stm32f407xx.h>
#include <string.h>

#define COMMAND1		0x01
#define COMMAND2		0x02
#define COMMAND3		0x03
#define COMMAND4		0x04

#define ACK				0xF5
#define NACK			0xA5

void delay(void)
{
	for(uint32_t i=0; i< 250000; i++);
}


SPI2_GPIOInits(void)
{
	GPIO_Handle_t 	SPIPins;
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

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_16;
	GPIO_Init(&SPIPins);
}

SPI2_Inits(void)
{
	SPI_Handle_t SPI_2;
	SPI_2.pSPIx = SPI2;
	SPI_2.SPIConfig.BusConfig = SPI_BUSCONFIG_FULLDUPLEX;
	SPI_2.SPIConfig.DeviceMode = SPI_DEVICEMODE_MASTER;
	SPI_2.SPIConfig.CPHA = SPI_CPHA_TRAILING_EDGE;
	SPI_2.SPIConfig.CPOL = SPI_CPOL_LOW_IDLE_STATE;
	SPI_2.SPIConfig.DFF = SPI_DFF_8BIT;
	SPI_2.SPIConfig.SclkSpeed = SPI_SCLKSPEED_FPCLK_8;
	SPI_2.SPIConfig.SSM = SPI_SSM_DI;

	SPI_Init(&SPI_2);

	while(1);

}

int main(void)
{


	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);






	return 0;
}
