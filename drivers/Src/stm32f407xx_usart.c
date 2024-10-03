/*
 * stm32f407xx_usart.c
 *
 *  Created on: Oct 3, 2024
 *      Author: Vishal Turaga
 */

#include "stm32f407x_usart.h"

/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_ONLY_TX					0
#define USART_MODE_ONLY_RX 					1
#define USART_MODE_TXRX  					2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD					2
#define USART_PARITY_EN_EVEN				1
#define USART_PARITY_DISABLE				0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS					0
#define USART_WORDLEN_9BITS					1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1					0
#define USART_STOPBITS_0_5					1
#define USART_STOPBITS_2					2
#define USART_STOPBITS_1_5					3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE		    	0
#define USART_HW_FLOW_CTRL_CTS		    	1
#define USART_HW_FLOW_CTRL_RTS		    	2
#define USART_HW_FLOW_CTRL_CTS_RTS			3


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



