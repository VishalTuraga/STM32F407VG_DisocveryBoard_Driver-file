/*
 * 005_STMMaster_ArduinoSlave.c
 *
 *  Created on: Sep 5, 2024
 *      Author: ASUS
 */


#include <stm32f407xx.h>
#include <string.h>

GPIO_Handle_t 	SPIPins;
SPI_Handle_t 	SPI_2;
GPIO_Handle_t Button;
GPIO_Handle_t LED;

void delay(void)
{
	for(uint32_t i=0; i< 250000; i++);
}

char user_data[] = "This is the data sent to the master everytime there is a button pressed";

void LED_Config(void)
{
	memset(&LED,0,sizeof(LED));
	LED.pGPIOx = GPIOD;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_LOW;

	GPIO_ClockControl(LED.pGPIOx, ENABLE);
	GPIO_Init(&LED);
}

void Button_Config(void)
{
	memset(&Button,0,sizeof(Button));
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Init(&Button);
}

void SPI2_GPIOInits(void)
{
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_2.pSPIx = SPI2;
	SPI_2.SPIConfig.BusConfig = SPI_BUSCONFIG_FULLDUPLEX;
	SPI_2.SPIConfig.DeviceMode = SPI_DEVICEMODE_MASTER;
	SPI_2.SPIConfig.CPHA = SPI_CPHA_TRAILING_EDGE;
	SPI_2.SPIConfig.CPOL = SPI_CPOL_LOW_IDLE_STATE;
	SPI_2.SPIConfig.DFF = SPI_DFF_8BIT;
	SPI_2.SPIConfig.SclkSpeed = SPI_SCLKSPEED_FPCLK_8;
	SPI_2.SPIConfig.SSM = SPI_SSM_DI;

	SPI_Init(&SPI_2);

}

int main(void)
{
	LED_Config();

	Button_Config();

	SPI2_GPIOInits();

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);

	GPIO_WritePin(GPIOD, GPIO_PIN_12, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_EXTI0, 1);
	GPIO_IRQITConfig(IRQ_EXTI0, ENABLE);

	while(1);

	return 0;
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_TogglePinOutput(GPIOD, GPIO_PIN_12);
	SPI_PeripheralControl(SPI2, ENABLE);

	// sending len info
	uint8_t stringlen = strlen(user_data);
	SPI_SendData(SPI2, &stringlen, 1);

	// sending datta
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
	if((SPI2->SR & (1 << SPI_SR_BSY)) == 0)
		SPI_PeripheralControl(SPI2, DISABLE);
}
