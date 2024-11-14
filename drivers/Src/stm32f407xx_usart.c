/*
 * stm32f407xx_usart.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Vishal Turaga
 */

#include "stm32f407x_usart.h"



/*************************************************************************************************
 * @fn				- USART_GetFlagStatus
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
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if((pUSARTx->SR1 & FlagName))
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*************************************************************************************************
 * @fn				- USART_ClearFlag
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
uint8_t USART_ClearFlag(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	pUSARTx->SR &= ~(1 << FlagName);
}

/*
 * Peripheral Clock setup
 */
/*************************************************************************************************
 * @fn				- USART_PeriClockControl
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
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_CLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_CLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_CLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_CLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_CLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_CLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_CLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_CLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_CLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_CLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_CLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_CLK_DI();
		}
	}
}

/*
 * Init and De-init
 */
/*************************************************************************************************
 * @fn				- USART_Init
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
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

	//Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


	//Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable ODD parity
		tempreg |= ( 1 << USART_CR1_PS);

	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= ((1 << USART_CR3_CTSE)| (1 << USART_CR3_RTSE));
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

}

/*************************************************************************************************
 * @fn				- USART_DeInit
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
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		RCC->APB2RSTR |= (1 << 4);
		RCC->APB2RSTR &= ~(1 << 4);
	}
	else if(pUSARTx == USART2)
	{
		RCC->APB1RSTR |= (1 << 17);
		RCC->APB1RSTR &= ~(1 << 17);
	}
	else if(pUSARTx == USART3)
	{
		RCC->APB1RSTR |= (1 << 18);
		RCC->APB1RSTR &= ~(1 << 18);
	}
	if(pUSARTx == UART4)
	{
		RCC->APB1RSTR |= (1 << 19);
		RCC->APB1RSTR &= ~(1 << 19);
	}
	else if(pUSARTx == UART5)
	{
		RCC->APB1RSTR |= (1 << 20);
		RCC->APB1RSTR &= ~(1 << 20);
	}
	else if(pUSARTx == USART6)
	{
		RCC->APB2RSTR |= (1 << 5);
		RCC->APB2RSTR &= ~(1 << 5);
	}
}


/*
 * Data Send and Receive
 */


/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 *********************************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			*pdata = (*pdata & (uint16_t)0x01FF);

			pUSARTHandle->pUSARTx->DR = (*pdata);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TODO;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TODO = Len;
		pUSARTHandle->pTxBuffer = TODO;
		pUSARTHandle->TxBusyState = TODO;

		//Implement the code to enable interrupt for TXE
		TODO


		//Implement the code to enable interrupt for TC
		TODO


	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->TODO;

	if(rxstate != TODO)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = TODO;
		pUSARTHandle->RxBusyState = TODO;

		//Implement the code to enable interrupt for RXNE
		TODO

	}
	return rxstate;
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Resolve all the TODOs

 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 *BaudRate));
	}else
	{
		//over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
	{
		//OVER8 = 1 , over sampling by 8
		F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

	}else
	{
		//over sampling by 16
		F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

	}

	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

/*
 * IRQ Configuration and ISR handling
 */
/*************************************************************************************************
 * @fn				- USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){}

/*************************************************************************************************
 * @fn				- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){}

/*************************************************************************************************
 * @fn				- USART_IRQHandling
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
void USART_IRQHandling(USART_Handle_t *pHandle){}

/*
 * Other Peripheral Control APIs
 */
/*************************************************************************************************
 * @fn				- USART_PeripheralControl
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
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pUSARTx->CR1 |= ( 1 << USART_CR1_UE);
	}
	else if(EnOrDi == DISABLE)
	{
		pUSARTx->CR1 &= ~( 1 << USART_CR1_UE);
	}
}

uint16_t AHBPreArr[9] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1PreArr[4] = {2,4,8,16};

/*************************************************************************************************
 * @fn				- RCC_GetPCLK1Value
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
uint32_t RCC_GetPCLK2Value()
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHBPreArr[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1PreArr[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}


/*************************************************************************************************
 * @fn				- RCC_GetPCLK1Value
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
		AHB1PRE = AHBPreArr[temp-8];
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
 * Application callback
 */
/*************************************************************************************************
 * @fn				- USART_ApplicationEventCallback
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
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv){}




