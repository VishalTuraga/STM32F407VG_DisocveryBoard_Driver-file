#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i=0; i< 100000; i++);
}

int main(void)
{
	GPIO_Handle_t LED;

 	LED.pGPIOx = GPIOD;
	LED.GPIO_PinConfig.GPIO_PinNumber 		= 	GPIO_PIN_12;
	LED.GPIO_PinConfig.GPIO_PinMode 		=	GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinOPType		=	GPIO_OUT_TYPE_PP;
	LED.GPIO_PinConfig.GPIO_PinSpeed		=	GPIO_OUT_SPEED_LOW;

	GPIO_ClockControl(LED.pGPIOx = GPIOD, ENABLE);

	GPIO_Init(&LED);

	while(1)
	{
		delay();
		GPIO_TogglePinOutput(LED.pGPIOx = GPIOD,LED.GPIO_PinConfig.GPIO_PinNumber);

	}



	return 0;
}
