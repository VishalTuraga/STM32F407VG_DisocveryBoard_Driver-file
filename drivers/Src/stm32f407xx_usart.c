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
/*************************************************************************************************
 * @fn				- USART_SendData
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
void USART_SendData(USART_RegDef_t *pUSARTx,uint8_t *pTxBuffer, uint32_t Len)
{

}

/*************************************************************************************************
 * @fn				- USART_ReceiveData
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
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
{
}

/*************************************************************************************************
 * @fn				- USART_SendDataIT
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
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){}

/*************************************************************************************************
 * @fn				- USART_ReceiveDataIT
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
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){}

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
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName){}

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
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){}

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



