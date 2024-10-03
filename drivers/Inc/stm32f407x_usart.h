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

#endif /* INC_STM32F407X_USART_H_ */
