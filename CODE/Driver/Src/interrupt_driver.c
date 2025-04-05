
#include <interrupt_driver.h>

//API IMPLEMENTATION

//Initialization and De-initialization
void GPIO_init(GPIO_HANDLE_t *pGPIOHandle){
	uint32_t temp=0;

	//PINMODE
			if(pGPIOHandle ->GPIO_CONFIG_t.PinMode <= GPIOMODE_Analog){
				temp=pGPIOHandle->GPIO_CONFIG_t.PinMode << (2*pGPIOHandle->GPIO_CONFIG_t.PinNumber);
				pGPIOHandle->pGPIOx->MODER &=~(0x3 << (2*pGPIOHandle->GPIO_CONFIG_t.PinNumber)); //CLEARING
				pGPIOHandle->pGPIOx->MODER |=temp;	//SETTING
			}
			else{    //INTERRUPT
		           if(pGPIOHandle->GPIO_CONFIG_t.PinMode==GPIOMODE_InterruptFT){
		                 EXTI->FTSR |=(1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);
		                 EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);
		           }
		           else if(pGPIOHandle->GPIO_CONFIG_t.PinMode==GPIOMODE_InterruptHT){
		        	   EXTI->RTSR |=(1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);
		        	   EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);

		           }
		           else if(pGPIOHandle->GPIO_CONFIG_t.PinMode==GPIOMODE_TRIGGER){
		        	   EXTI->RTSR |=(1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);
		        	   EXTI->FTSR |= (1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);
		           }

		           //ENABLE SYSCFG CLOCK
		           uint32_t *RCC_SYSCFG1=(uint32_t*)0x40023844;
		          *RCC_SYSCFG1=*RCC_SYSCFG1 | (1<<14);

		           //GPIO PORT SELECTION
		           uint8_t temp1=pGPIOHandle->GPIO_CONFIG_t.PinNumber/4;
		           uint8_t temp2=pGPIOHandle->GPIO_CONFIG_t.PinNumber%4;
		           uint8_t portcode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		           SYSCFG->EXTICR[temp1]=portcode <<(temp2*4);

		           //ENABLE EXTI INTERRUPT DELIVERY
		           EXTI->IMR |=(1<<pGPIOHandle->GPIO_CONFIG_t.PinNumber);

			}

	//PIN SPEED
	temp=(pGPIOHandle->GPIO_CONFIG_t.PinSpeed <<(2*pGPIOHandle->GPIO_CONFIG_t.PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0X3 << (2*pGPIOHandle->GPIO_CONFIG_t.PinNumber)); //CLEARING
	pGPIOHandle->pGPIOx->OSPEEDER |=temp; //SETTING


	//PIN PUPD CONDTROL
	temp=(pGPIOHandle->GPIO_CONFIG_t.PinPuPdControl <<(2*pGPIOHandle->GPIO_CONFIG_t.PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_CONFIG_t.PinNumber)); //CLEARING
	pGPIOHandle->pGPIOx->PUPDR |=temp; //SETTING


	//PIN OUTPUT TYPE
	temp=(pGPIOHandle->GPIO_CONFIG_t.PinOPType <<(pGPIOHandle->GPIO_CONFIG_t.PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &=~(1 << (pGPIOHandle->GPIO_CONFIG_t.PinNumber)); //CLEARING
	pGPIOHandle->pGPIOx->OTYPER |=temp; //SETTING


	//ALTERNATE FUNCTION
	if(pGPIOHandle->GPIO_CONFIG_t.PinMode ==GPIOMODE_AltFunction){
		uint8_t temp1,temp2;

		temp1= pGPIOHandle->GPIO_CONFIG_t.PinNumber/8;
		temp2= pGPIOHandle->GPIO_CONFIG_t.PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF << (4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_CONFIG_t.PinAltFunMode << (4*temp2));

	}
}


//INPUT
uint8_t GPIO_INPUT_PINREAD(GPIO_REGDEF_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1);
	return value;
}
uint16_t GPIO_INPUT_PORTREAD(GPIO_REGDEF_t *pGPIOx){
	uint16_t value;
		value=(uint16_t)(pGPIOx->IDR);
		return value;
}

//OUTPUT
void GPIO_OUTPUT_PINWRITE(GPIO_REGDEF_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
     if(Value == 1){
    	 pGPIOx->ODR |=(1<<PinNumber);
     }else{
    	 pGPIOx->ODR &=~(1<<PinNumber);
     }
}
void GPIO_OUTPUT_PORTWRITE(GPIO_REGDEF_t *pGPIOx,uint16_t Value){
	pGPIOx->ODR=Value;
}
void GPIO_OUTPUT_TOGGLEPIN(GPIO_REGDEF_t *pGPIOx,uint8_t PinNumber){
	 pGPIOx->ODR ^=(1<<PinNumber);
}

//INTERRUPT
void GPIO_IRQCONFIG(uint8_t IRQNO,uint8_t EnableDisable){
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
void GPIO_IRQPriority(uint32_t IRQNumber,uint8_t Priority){

	uint8_t irp=IRQNumber/4;
	uint8_t irp_section=IRQNumber%4;
	uint8_t Shift=(8*irp_section)+4;

	*(NVIC_Priority +(irp*4))|=(Priority<<Shift);

}

void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & (1<<PinNumber)){
		EXTI->PR |= (1<<PinNumber);
	}
}

