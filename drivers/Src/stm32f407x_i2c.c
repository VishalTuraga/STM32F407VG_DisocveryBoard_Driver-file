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
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);
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

	//5. Configure the rise time for I2C pins
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM_KHZ)
	{
		// mode is standard
		// here trsie = (Fclk * Trisemax) + 1.
		// For I2C, trisemax is 1000ns or 1microsecond and in frequency terms it is 1 MHz. Hence we are dividing with 1 MHz
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// mode is fast mode
		// trise max for fast mode is 300ns
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);

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
	if(pI2Cx == I2C1)
	{
		I2C1_CLK_DI();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_CLK_DI();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_CLK_DI();
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if((pI2Cx->SR1 & FlagName) || (pI2Cx->SR2 & FlagName))
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * I2C send and receive data
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *TxBuffer, uint8_t len, uint8_t SlaveAddr)
{
	uint32_t temp;
	// 1. Generate the start condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	// 2. Check if the start bit is set and then Read the SR1 register to clear the start bit
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SR1_SB) == FLAG_SET));
	//uint32_t temp = pI2CHandle->pI2Cx->SR1;

	// 3. Send the address of slave with transmission byte (0)
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2CHandle->pI2Cx->DR = SlaveAddr;

	// 4. ADDR bit is set if it receives an ACK
	if(I2C_GetFlagStatus(pI2CHandle->pI2Cx,(1 << I2C_SR1_ADDR)))
	{
		// The ADDR bit is set which means that the master received an ack. Now we should reset this ADDR bit
		// read SR1 and SR2 to clear this bit
		temp = pI2CHandle->pI2Cx->SR1;
		temp = pI2CHandle->pI2Cx->SR2;
		(void)temp;
	}

	// 5. Send data till len becomes zero. We don't have to check for ack every time as it is handled by the hardware
	while(len)
	{
		// wait till Txe is 1 indicating that DR is empty and ready to be filled with data
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,(1 << I2C_SR1_TxE))))
		{
			pI2CHandle->pI2Cx->DR = *TxBuffer;
			TxBuffer++;
			len--;
		}
	}

	// 6. Close the communication
	// 6.1 wait for Txe = 1 and BTF = 1 before generating the stop condition
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,(1 << I2C_SR1_TxE))));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,(1 << I2C_SR1_BTF))));

	// 6.2 Generate the stop condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
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

/*
 * I2C send and receive data
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *RxBuffer, uint8_t len, uint8_t SlaveAddr)
{
	uint32_t temp;
	// 1. Initiate the start condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	// 2. Confirm if the start bit is set
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SR1_SB) == FLAG_SET));

	// 3. Send Address bit
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
	pI2CHandle->pI2Cx->DR = SlaveAddr;

	// 4. check if the ADDR flag is set. Wait until its set
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx,(1 << I2C_SR1_ADDR))));

	// 5. Send data. If len = 1 or if len > 1
	if(len == 1)
	{
		// a. Disable ack
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

		// c. clear addr bit. Read SR1 followed by SR2
		temp = pI2CHandle->pI2Cx->SR1;
		temp = pI2CHandle->pI2Cx->SR2;
		(void)temp;

		// d. wait till RXNE is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_RxNE)));

		// b. send stop condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);

		// e. Read the data from the data register
		*RxBuffer = pI2CHandle->pI2Cx->DR;
		RxBuffer++;
		len--;

	}
	if(len > 0)
	{
		// a. clear the address bit
		temp = pI2CHandle->pI2Cx->SR1;
		temp = pI2CHandle->pI2Cx->SR2;
		(void)temp;

		// b. receive data
		while(len > 0)
		{
			// d. wait till RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, (1 << I2C_SR1_RxNE)));

			if(len == 2)
			{
				// c. when len == 2,
				// c.1 clear ACK
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);

				// c.2 set STOP condition
				pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
			}

			*RxBuffer = pI2CHandle->pI2Cx->DR;

			len--;
			RxBuffer++;
		}

	}

	// 7. Renable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACKCTRL_ACK_EN)
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);

}

}
