/*
 * Usart_tx.c
 *
 *  Created on: Feb 28, 2025
 *      Author: Ocean
 */


//PA9-D8-TX
//PA10-RX-D2
#include<stdint.h>
#include<stm.h>
#include<string.h>

void delay(){
	for(uint32_t i =0;i<500000/2;i++);
}


char msg[1024]="UART Tx Testing..\n\r";
USART_Handle_t USARTx;


void Button_Init(void){
	//Button
		GPIO_HANDLE_t GpioButton;
		 GpioButton.pGPIOx=GPIOC;
		 GpioButton.GPIO_CONFIG_t.PinNumber=13;
		 GpioButton.GPIO_CONFIG_t.PinMode=GPIOMODE_INPUT;
        GpioButton.GPIO_CONFIG_t.PinSpeed=SPEED_HIGH;
        GpioButton.GPIO_CONFIG_t.PinPuPdControl=NOPULL;
		GPIO_init(&GpioButton);
}

void USART1_Init(void){

	USARTx.USART_Config.Mode=USART_MODE_ONLY_TX;
	USARTx.USART_Config.BaudRate=USART_STD_BAUD_115200;
	USARTx.USART_Config.ParityControl=USART_PARITY_DISABLE;
	USARTx.USART_Config.WordLength=USART_WORDLEN_8BITS;
	USARTx.USART_Config.StopBits=USART_STOPBITS_1;
	USARTx.USART_Config.HW_FlowControl=USART_HW_FLOW_CTRL_NONE;

	//USART1
	USARTx.pUSARTx=USART1;
	USART_Init(&USARTx);
}

void USART_GPIO_Init(void){

	GPIO_HANDLE_t USART_Pin;
	USART_Pin.pGPIOx=GPIOA;
	USART_Pin.GPIO_CONFIG_t.PinMode=GPIOMODE_AltFunction;
	USART_Pin.GPIO_CONFIG_t.PinAltFunMode=7;
	USART_Pin.GPIO_CONFIG_t.PinOPType=OUTPUT_PUSHPULL;
	USART_Pin.GPIO_CONFIG_t.PinSpeed=SPEED_HIGH;
	USART_Pin.GPIO_CONFIG_t.PinPuPdControl=PULLUP;

	//PA9-D8-TX
	USART_Pin.GPIO_CONFIG_t.PinNumber=9;
	GPIO_init(&USART_Pin);

	//PA10-RX-D2
	USART_Pin.GPIO_CONFIG_t.PinNumber=10;
	GPIO_init(&USART_Pin);
}
int main(void){
	//RCC GPIO
		uint32_t *RCC_AHB1 =(uint32_t*)0x40023830;
		*RCC_AHB1 &=~(1<<1);
		*RCC_AHB1 &=~(1<<0);
		*RCC_AHB1 &=~(1<<2);

		*RCC_AHB1 |=(1<<1);
		*RCC_AHB1 |=(1<<0);
		*RCC_AHB1 |=(1<<2);

		//RCC USART
			uint32_t *RCC_APB2=(uint32_t*)0x40023844;
			*RCC_APB2 &=~(1<<4);
			*RCC_APB2 |=(1<<4);


	Button_Init();
     USART_GPIO_Init();
     USART1_Init();
     USART_PeripheralControl(USART1,1);

	while(1){
    //Button Press
			while(GPIO_INPUT_PINREAD(GPIOC,13));
			delay();
	//USART DATA TRANSFER
			USART_SendData(&USARTx,(uint8_t*)msg,strlen(msg));
	}

     return 0;

}
