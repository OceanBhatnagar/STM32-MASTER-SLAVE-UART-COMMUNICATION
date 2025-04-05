/*
 * usart_driver.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Ocean
 */

#include<usart_driver.h>

void USART_SetBaudRate(USART_REGDEF_t *pUSARTx, uint32_t BaudRate)
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
	   PCLKx = 16000000;
  }else
  {
	   PCLKx = 16000000;
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->USART_CR1& (1 <<15))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->USART_CR1 & ( 1 << 15))
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
  pUSARTx->USART_BRR = tempreg;
}



void USART_PeripheralControl(USART_REGDEF_t *pUSARTx,uint8_t EnorDi){
	if(EnorDi==1){
		pUSARTx->USART_CR1 |=(1<<13);
		}
		else{
			pUSARTx->USART_CR1 &=~(1<<13);
		}
}


uint8_t USART_FlagStatus(USART_REGDEF_t *pUSARTx,uint8_t StatusFlagName){
	if(pUSARTx->USART_SR & StatusFlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void USART_ClearFlag(USART_REGDEF_t *pUSARTx,uint16_t StatusFlagName)
{
	pUSARTx->USART_SR &= ~( StatusFlagName);

}

void USART_Init(USART_Handle_t *pUSARTHandle){
	//CR1
	uint32_t temp=0;

	//CLOCK ENABLE

	//USART MODE
	if(pUSARTHandle->USART_Config.Mode==USART_MODE_ONLY_RX){
		//Receiver Enable
		temp |=(1<<2);
	}
	else if(pUSARTHandle->USART_Config.Mode==USART_MODE_ONLY_TX){
		//Transmitter Enable
		temp |=(1<<3);
	}
	else if(pUSARTHandle->USART_Config.Mode==USART_MODE_TXRX){
		temp |=((1<<2) | (1<<3));
	}

	//USART WordLength
	temp |=pUSARTHandle->USART_Config.WordLength<<12;

	//PARITY CONTROL
	if(pUSARTHandle->USART_Config.ParityControl==USART_PARITY_EN_EVEN){
		temp |=(1<<10);
		temp|=(1<<9);
	}

	pUSARTHandle->pUSARTx->USART_CR1=temp;

	//CR2

	temp=0;
	//Stop Bits
	temp |=pUSARTHandle->USART_Config.StopBits<<12;
	pUSARTHandle->pUSARTx->USART_CR2=temp;

	//CR3
	temp=0;

	//Hardware Flow Control
	if(pUSARTHandle->USART_Config.HW_FlowControl==USART_HW_FLOW_CTRL_CTS){
		temp |=(1<<9);
	}
	else if(pUSARTHandle->USART_Config.HW_FlowControl==USART_HW_FLOW_CTRL_RTS){
		temp |=(1<<8);
	}
	else if(pUSARTHandle->USART_Config.HW_FlowControl==USART_HW_FLOW_CTRL_CTS_RTS){
		temp |=(1<<9);
		temp |=(1<<8);
	}

	temp |=pUSARTHandle->pUSARTx->USART_CR3;
	//BAUD RATE
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.BaudRate);
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_FlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_DISABLE)
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
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_FlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_TC ));
}


void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_FlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE ));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
			    pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR &(uint8_t)0x7F);

			}

			//increment the pRxBuffer
			pRxBuffer++;
		}
	}

}

//*************************************************************************************************************



void USART_IRQCONFIG(uint8_t IRQNO,uint8_t EnableDisable){
if(EnableDisable==1){
		if(IRQNO <=31){
			*NVIC_ISER0 |=(1<<IRQNO);
		}
		else if(IRQNO>31 && IRQNO<64){
			*NVIC_ISER1 |=(1<<(IRQNO%32));
		}
		else if(IRQNO>=64 && IRQNO<96){
			*NVIC_ISER2 |=(1<<(IRQNO%64));
		}
	}
	else{
		if(IRQNO <=31){
					*NVIC_ICER0 |=(1<<IRQNO);
				}
				else if(IRQNO>31 && IRQNO<64){
					*NVIC_ICER1 |=(1<<(IRQNO%32));
				}
				else if(IRQNO>=64 && IRQNO<96){
					*NVIC_ICER2 |=(1<<(IRQNO%64));
				}
			}
	}

void USART_IRQPriority(uint32_t IRQNumber,uint8_t Priority){
	uint8_t irp=IRQNumber/4;
		uint8_t irp_section=IRQNumber%4;
		uint8_t Shift=(8*irp_section)+4;

		*(NVIC_Priority +(irp*4))|=(Priority<<Shift);
}


uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState =  USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<7);
		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<6);


	}

	return txstate;

}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState =USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->USART_DR;
		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->USART_CR1 |=(1<<5);

	}

	return rxstate;

}
