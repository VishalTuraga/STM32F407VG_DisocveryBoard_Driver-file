/*
 * stm32f407x_i2c.c
 *
 *  Created on: Sep 15, 2024
 *      Author: Vishal Turaga
 */


#include "stm32f407xx_i2c.h"

/*
 * Enable/Disable I2C Peripheral
 */
/*************************************************************************************************
 * @fn				- I2C_PeriheralControl
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in[		-
 *
 * @return			-
 *
 * @Note			-
 *
 *************************************************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else if(EnOrDi == DISABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
}

void I2C_ClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_CLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_CLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_CLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_CLK_DI();
		}
	}
}

uint16_t AHB1PreArr[9] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1PreArr[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1;
	// 1. Get the clock source
	uint8_t clocksource, temp, AHB1PRE, APB1PRE;

	uint32_t ClkSrcFreq;
	clocksource = ((RCC->CFGR >> 2) & 0x3);
	if(clocksource == 0)
	{
		// HSI is used. Freq = 16LHz
		ClkSrcFreq = 16000000;
	}
	else if(clocksource == 1)
	{
		// HSE is used. Freq = 8MHz
		ClkSrcFreq = 8000000;
	}
	//	else if(clocksource == 2)
	//	{
	//		// PLL is used
	//	}

	// 2. Get the AHB prescalar
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		AHB1PRE = 1;
	}
	else
	{
		AHB1PRE = AHB1PreArr[temp-8];
	}

	// 3. Get the APB1 Prescalar
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		APB1PRE = 1;
	}
	else
	{
		APB1PRE = APB1PreArr[temp-4];
	}

	pclk1 = ((ClkSrcFreq / AHB1PRE) / APB1PRE);

	return pclk1;
}

/*
 * I2C Init and De-Init
 */
/*************************************************************************************************
 * @fn				- I2C_Init
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in[		-
 *
 * @return			-
 *
 * @Note			-
 *
 *************************************************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;
	//1. Configure the mode (Standard or fast)
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM_KHZ)
	{
		// mode is standard
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// mode is fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_DUTYCYCLE_2)
		{
			// Tlow = 2*Thigh
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));

		}
		else
		{
			//  Tlow = 1.7*Thigh
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//2. Configure the speed of the serial clock. Configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	//3. Configure the device address (if the device is behaving as slave)
	// configuring for only 7 bit mode
	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	pI2CHandle->pI2Cx->OAR1 |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//4. Enable the acking
	pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);

	//5. Configure the rise time for I2C pins (will discuss later)

	// CCR calculations


}

/*************************************************************************************************
 * @fn				- I2C_Deinit
 *
 * @brief			-
 *
 * @param[in]		-
 * @param[in]		-
 * @param[in[		-
 *
 * @return			-
 *
 * @Note			-
 *
 *************************************************************************************************/
void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{

}
