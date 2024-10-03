/*
 * stm32f407x_usart.h
 *
 *  Created on: Oct 2, 2024
 *      Author: ASUS
 */

#ifndef INC_STM32F407X_USART_H_
#define INC_STM32F407X_USART_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t 	USART_Mode;
	uint32_t	USART_Baud;
	uint8_t		USART_NoOfStopBits;
	uint8_t		USART_WordLength;
	uint8_t		USART_ParityControl;
	uint8_t		USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle Structure for USARTx peripheral
 */
typedef struct
{
	USART_Config_t USART_Config;
	USART_RegDeg_t *pUSARTx;
}USART_Handle_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USART_RegDef_t *pUSARTx,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F407X_USART_H_ */
