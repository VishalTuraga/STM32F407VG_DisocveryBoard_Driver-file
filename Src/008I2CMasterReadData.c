/*
 * 008I2CMasterReadData.c
 *
 *  Created on: Sep 29, 2024
 *      Author: Vishal Turaga
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"

void ButtonGPIOInits(void);
void I2C1GPIOInits(void);
void I2C1Inits(void);

uint8_t CMDReadLen	= 0x51;
uint8_t CMDReadData	= 0x52;

#define MYADDR		0x68
#define SLAVEADDR	0x61

#define MAXLEN		1024
uint8_t buffer[MAXLEN];

uint8_t length;

I2C_Handle_t I2C1Handle;



int main(void)
{
	I2C1GPIOInits();

	I2C1Inits();

	ButtonGPIOInits();

	GPIO_IRQITConfig(IRQ_EXTI0, ENABLE);

	/*
	 * if we are not using button interrupt then the code will look like this
	 * I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);
	 * while(1)
	 * {
	 * 		I2C_MasterSendData(&I2C1Handle, CMDReadLen, 1, SLAVEADDR);

			I2C_MasterReceiveData(&I2C1Handle, &length, 1, SLAVEADDR);

			I2C_MasterSendData(&I2C1Handle, CMDReadData, 1, SLAVEADDR);

			I2C_MasterReceiveData(&I2C1Handle, buffer, length, SLAVEADDR);
	 * }
	 * I2C_PeripheralControl(I2C1Handle, DISABLE);
	 */

	printf("Data received: %s\n",buffer);

	while(1);

	return 0;
}

void ButtonGPIOInits(void)
{
	GPIO_Handle_t Button;
	memset(&Button,0,sizeof(Button));
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_HIGH;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Init(&Button);
}

void I2C1GPIOInits(void)
{
	GPIO_Handle_t I2C1GPIO;
	memset(&I2C1GPIO,0,sizeof(I2C1GPIO));
	I2C1GPIO.pGPIOx = GPIOB;
	I2C1GPIO.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2C1GPIO.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALTFN_AF4;
	I2C1GPIO.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OUT_SPEED_LOW;
	I2C1GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_OD;
	I2C1GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PULLUP;

	I2C1GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2C1GPIO);

	I2C1GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2C1GPIO);
}

void I2C1Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACKCTRL_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x68;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM_KHZ;

}

void EXTI0_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_0);

	I2C_PeripheralControl(I2C1Handle.pI2Cx, ENABLE);

	I2C_ManageAcking(I2C1Handle.pI2Cx, ENABLE);

	I2C_MasterSendData(&I2C1Handle, &CMDReadLen, 1, SLAVEADDR, I2C_SR);

	I2C_MasterReceiveData(&I2C1Handle, &length, 1, SLAVEADDR, I2C_SR);

	I2C_MasterSendData(&I2C1Handle, &CMDReadData, 1, SLAVEADDR,I2C_SR);

	I2C_MasterReceiveData(&I2C1Handle, buffer, length, SLAVEADDR, I2C_NO_SR);

	I2C_PeripheralControl(I2C1Handle.pI2Cx, DISABLE);
}
