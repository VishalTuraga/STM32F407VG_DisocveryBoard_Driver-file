#include "stm32f407xx.h"
#include <string.h>

void delay(void)
{
	for(uint32_t i=0; i< 250000; i++);
}
GPIO_Handle_t LED;
GPIO_Handle_t Button;

void LED_Config(void);
void Button_Config(void);

int main(void)
{
	LED_Config();
	Button_Config();
	while(1)
	{
		if(GPIO_ReadPin(Button.pGPIOx, Button.GPIO_PinConfig.GPIO_PinNumber) == GPIO_PIN_SET)
		{
			delay();
			GPIO_TogglePinOutput(LED.pGPIOx, LED.GPIO_PinConfig.GPIO_PinNumber);
		}
	}

	GPIO_DeInit(GPIOA);
	GPIO_DeInit(GPIOD);



	return 0;
}

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
	Button.GPIO_PinConfig.GPIO_PinNumber		=	GPIO_PIN_0;
	Button.GPIO_PinConfig.GPIO_PinMode			= 	GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl	=	GPIO_PUPD_NONE;

	GPIO_ClockControl(Button.pGPIOx, ENABLE);
	GPIO_Init(&Button);
}
