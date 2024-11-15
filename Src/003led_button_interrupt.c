#include "stm32f407xx.h"
#include <string.h>

void LED_Config(void);
void Button_Config(void);

void delay(void)
{
	for(int i=0;i<250000;i++);
}

GPIO_Handle_t LED;


int main()
{
	// PD 12 is the LED
	// PA 0 is the user button

	// LED configuration
	LED_Config();

	// Button configuration
	Button_Config();

	// Initialize Interrupt
	GPIO_WritePin(GPIOD, GPIO_PIN_12, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_EXTI0, 1);
	GPIO_IRQITConfig(IRQ_EXTI0, ENABLE);

	while(1);

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
	GPIO_Handle_t Button;
	memset(&Button,0,sizeof(Button));
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_ClockControl(Button.pGPIOx, ENABLE);
	GPIO_Init(&Button);
}

void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_0);
	GPIO_TogglePinOutput(GPIOD, GPIO_PIN_12);

}
