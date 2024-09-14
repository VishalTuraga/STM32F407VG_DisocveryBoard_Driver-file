#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"
#include <string.h>

/*
 * Pin B13: SCLK
 * Pin B15: MOSI
 */

GPIO_Handle_t SPI2_SCK;
GPIO_Handle_t SPI2_MOSI;
GPIO_Handle_t SPI2_MISO;
GPIO_Handle_t SPI2_NSS;

SPI_Handle_t SPI_2;

void SPI2_GPIOInits(void)
{
	SPI2_SCK.pGPIOx = GPIOB;
	SPI2_SCK.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	SPI2_SCK.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_SCK.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF5;
	SPI2_SCK.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;

	SPI2_MOSI.pGPIOx = GPIOB;
	SPI2_MOSI.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	SPI2_MOSI.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_MOSI.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF5;
	SPI2_MOSI.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;

	SPI2_MISO.pGPIOx = GPIOB;
	SPI2_MISO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	SPI2_MISO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_MISO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF5;
	SPI2_MISO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;

	SPI2_NSS.pGPIOx = GPIOB;
	SPI2_NSS.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	SPI2_NSS.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI2_NSS.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF5;
	SPI2_NSS.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;

	GPIO_Init(&SPI2_NSS);
	GPIO_Init(&SPI2_MISO);
	GPIO_Init(&SPI2_MOSI);
	GPIO_Init(&SPI2_SCK);
}

void SPI2_Init(void)
{
	SPI_2.pSPIx = SPI2;
	SPI_2.SPIConfig.BusConfig = SPI_BUSCONFIG_FULLDUPLEX;
	SPI_2.SPIConfig.DeviceMode = SPI_DEVICEMODE_MASTER;
	SPI_2.SPIConfig.CPHA = SPI_CPHA_TRAILING_EDGE;
	SPI_2.SPIConfig.CPOL = SPI_CPOL_LOW_IDLE_STATE;
	SPI_2.SPIConfig.DFF = SPI_DFF_8BIT;
	SPI_2.SPIConfig.SclkSpeed = SPI_SCLKSPEED_FPCLK_2;
	SPI_2.SPIConfig.SSM = SPI_SSM_EN;

	SPI2->CR1 |= (1 << SPI_CR1_SSI);

	SPI_Init(&SPI_2);
}

int main(void)
{
	SPI2_GPIOInits();

	SPI2_Init();

	SPI_PeriheralControl(SPI2, ENABLE);

	char user_data[] = "Hello World";

	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	while(1);

	SPI_PeriheralControl(SPI2, DISABLE);

	return 0;
}
