/*
 * usart_driver.h
 *
 *  Created on: Feb 27, 2025
 *      Author: Ocean
 */

#ifndef INC_USART_DRIVER_H_
#define INC_USART_DRIVER_H_

#include<stm.h>
#include<stdint.h>

//USART PIN CONFIGURATION

typedef struct{
	uint8_t Mode;
	uint32_t BaudRate;
	uint8_t StopBits;
	uint8_t WordLength;
	uint8_t ParityControl;
	uint8_t HW_FlowControl;
}USART_Config_t;

typedef struct{
	USART_REGDEF_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
    uint8_t RxBusyState;
}USART_Handle_t;

//MODE
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

//Baud Rate
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

//Parity
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

//WordLegth
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

//Stop Bits
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

//HW Flow Control
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

#define USART_FLAG_TXE 			( 1 << 7)
#define USART_FLAG_RXNE 		( 1 << 5)
#define USART_FLAG_TC 			( 1 << 6)

#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

//API PROTOTYPE

//Peripheral Enable
void USART_PeripheralControl(USART_REGDEF_t *pUSARTx,uint8_t EnorDi);

//FLAGS
uint8_t USART_FlagStatus(USART_REGDEF_t *pUSARTx,uint8_t StatusFlagName);
void USART_ClearFlag(USART_REGDEF_t *pUSARTx,uint16_t StatusFlagName);

//INTERRUPT
void USART_IRQCONFIG(uint8_t IRQNO,uint8_t EnableDisable);
void USART_IRQPriority(uint32_t IRQNumber,uint8_t Priority);

//Init
void USART_Init(USART_Handle_t *pUSARTHandle);

//Data Send and Receive
void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//Application CallBack
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_USART_DRIVER_H_ */
