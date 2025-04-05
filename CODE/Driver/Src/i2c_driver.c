/*
 * i2c_driver.c
 *
 *  Created on: Feb 10, 2025
 *      Author: Ocean
 */
#include<i2c_driver.h>
#include<stdint.h>

static void I2C_START(I2C_REGDEF_t *pI2Cx);
static void I2C_EXECUTE_BASE_ADDRESS(I2C_REGDEF_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_EXECUTE_ADDRESS_READ(I2C_REGDEF_t *pI2Cx,uint8_t SlaveAddr);
static void ClearADDRFlag(I2C_REGDEF_t *pI2Cx);
static void I2C_STOP(I2C_REGDEF_t *pI2Cx);

static void I2C_START(I2C_REGDEF_t *pI2Cx){
	pI2Cx->CR1 |=(1<<8);
}
static void I2C_STOP(I2C_REGDEF_t *pI2Cx){
	pI2Cx->CR1 |=(1<<9);
}
static void I2C_EXECUTE_ADDRESS_READ(I2C_REGDEF_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr=SlaveAddr <<1;
	SlaveAddr |=(1);  //SlaveAddr=SLAVE ADDRESS+READ
	pI2Cx->DR=SlaveAddr;
}

static void I2C_EXECUTE_BASE_ADDRESS(I2C_REGDEF_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr=SlaveAddr <<1;
	SlaveAddr &=~(1);  //SlaveAddr=SLAVE ADDRESS+READ AND WRITE
	pI2Cx->DR=SlaveAddr;
}

static void ClearADDRFlag(I2C_REGDEF_t *pI2Cx){
	uint32_t dummyRead=pI2Cx->SR1;
     dummyRead=pI2Cx->SR2;
	(void)dummyRead;
}

void I2C_PeripheralControl(I2C_REGDEF_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi==1){
		pI2Cx->CR1 |=(1<<0);
	}
	else{
		pI2Cx->CR1 &=~(1<<0);
	}
}

void I2C_ACK_CONTROL(I2C_REGDEF_t *pI2Cx,uint8_t EnorDi){
	if(EnorDi==1){
		//Enable Acking
		pI2Cx->CR1 |=(1<<10);
	}
	else{
		//Disable Acking
		pI2Cx->CR1 &=~(1<<10);
	}
}



uint32_t RCC_GetPCLK1Value=16000000;

void I2C_init(I2C_Handle_t *I2CHandle){

	//RCC I2C
			uint32_t *RCC_APB1=(uint32_t*)0x40023840;
			*RCC_APB1 &=~(1<<21);
			*RCC_APB1 |=(1<<21);
	//Enable Acking
	uint32_t temp=0;
	temp |=(I2CHandle->I2CConfig.AckControl <<10);
	I2CHandle->pI2Cx->CR1=temp;

	//FREQ FIELD OF CR2
	temp=0;
	temp |=16;  //1000000U
	I2CHandle->pI2Cx->CR2=(temp & 0x3F);

	//SLAVE ADDRESS
	temp=0;
	temp |=(I2CHandle->I2CConfig.DeviceAddress<<1);
	temp |=(1<<14);
	I2CHandle->pI2Cx->OAR1=temp;

	//SERIAL CLOCK SPEED
	uint16_t ccr=0;
	temp=0;
	if(I2CHandle->I2CConfig.SCLSpeed<=I2C_SCL_SPEED_NORMAL){
		//Standard Mode
		ccr=RCC_GetPCLK1Value/(2*I2CHandle->I2CConfig.SCLSpeed);
		temp |=(ccr & 0xFFF);
	}
	else{
		//FAST MODE
		temp |=(1<<15);
		temp |=(I2CHandle->I2CConfig.FM_DutyCycle << 14);
		if(I2CHandle->I2CConfig.FM_DutyCycle==I2C_DUTYCYCLE_2)
		{
			ccr=RCC_GetPCLK1Value/(3*I2CHandle->I2CConfig.SCLSpeed);
		}
		else{
			ccr=RCC_GetPCLK1Value/(25*I2CHandle->I2CConfig.SCLSpeed);
		}
		temp |=(ccr & 0xFFF);
	}
	I2CHandle->pI2Cx->CCR=temp;

	//T_RISE CONFIGURATION

	if(I2CHandle->I2CConfig.SCLSpeed<=I2C_SCL_SPEED_NORMAL){
			//STANDARD MODE
			temp=(RCC_GetPCLK1Value/1000000U)+1;

		}
	else{
		//FAST MODE
		temp=((RCC_GetPCLK1Value*300)/1000000000U)+1;
	}
	I2CHandle->pI2Cx->TRISE=(temp & 0x3F);

}


void I2C_Master_DataSend(I2C_Handle_t *I2CHandle,uint8_t *pTxBuffer,uint32_t Len,uint8_t SlaveAddr){
	//GENERATE START CONDITION
	 I2C_START(I2CHandle->pI2Cx);

	 //CONFIRM START GENERATION
	 while(! I2C_GETFLAG(I2CHandle->pI2Cx, I2C_SB_FLAG));

	 //SEND THE ADDRESS OF SLAVE WITH W=0
	 I2C_EXECUTE_BASE_ADDRESS(I2CHandle->pI2Cx,SlaveAddr);

	 //CONFIRM ADDRESS PHASE
	 while(! I2C_GETFLAG(I2CHandle->pI2Cx, I2C_ADDR_FLAG));

	 //CLEAR ADDR FLAG
	 ClearADDRFlag(I2CHandle->pI2Cx);

	 //SEND DATA UNTIL LEN=0

	 while(Len>0){
		 while(!I2C_GETFLAG(I2CHandle->pI2Cx,I2C_TXE_FLAG));
		 I2CHandle->pI2Cx->DR=*pTxBuffer;
		 pTxBuffer++;
		 Len--;
	 }

	 //Wait For TXE=1 and BTF=1

	 while(!I2C_GETFLAG(I2CHandle->pI2Cx,I2C_TXE_FLAG));
	 while(!I2C_GETFLAG(I2CHandle->pI2Cx,I2C_BTF_FLAG));

	 //STOP CONDITION
	 I2C_STOP(I2CHandle->pI2Cx);

}

uint8_t I2C_GETFLAG(I2C_REGDEF_t *pI2Cx,uint32_t FlagName){
	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_MasterReceiveData(I2C_Handle_t *I2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{

//1.Generate START condition
	 I2C_START(I2CHandle->pI2Cx);
//2.confirm that start generation was completed
	 while(! I2C_GETFLAG(I2CHandle->pI2Cx, I2C_SB_FLAG));

//3. send the addrress of the slave with r/nw bit set to R(1) (8bits)
	 I2C_EXECUTE_ADDRESS_READ(I2CHandle->pI2Cx,SlaveAddr);

//4. wait until address is compleate by checking ADDR flag in SR1
	 while(! I2C_GETFLAG(I2CHandle->pI2Cx, I2C_ADDR_FLAG));
if(Len ==1)
{
// Read a single byte from Slave

//1. Disable Acking
	I2C_ACK_CONTROL(I2CHandle->pI2Cx,0);



//2.Clear the ADDR flag
	ClearADDRFlag(I2CHandle->pI2Cx);

//3.wait until RXNE = 1
	while(!I2C_GETFLAG(I2CHandle->pI2Cx,I2C_RXE_FLAG));
//4.Generate STOP condition
    I2C_STOP(I2CHandle->pI2Cx);

//.5Read data in the buffer DR
	*pRxBuffer=I2CHandle->pI2Cx->DR;

}
if(Len>1)
{

//read multiple bytes

// clear ADDR flag
	ClearADDRFlag(I2CHandle->pI2Cx);

//read the data until Len is zero

for(uint32_t i = Len; i > 0; i--)

{

//wait until RXNE = 1
	while(!I2C_GETFLAG(I2CHandle->pI2Cx,I2C_RXE_FLAG));

if(i == 2) // last 2 bytes remaining

{

//Disable Acking
	I2C_ACK_CONTROL(I2CHandle->pI2Cx,0);
//generate STOP condition
	I2C_STOP(I2CHandle->pI2Cx);
}

//read the data from data register DR into buffer
*pRxBuffer=I2CHandle->pI2Cx->DR;
// increment the buffer address
pRxBuffer++;
}
}
//re-enable ACKing
if(I2CHandle->I2CConfig.AckControl==1){
I2C_ACK_CONTROL(I2CHandle->pI2Cx,1);
}
}


//SLAVE MODE

void I2C_Slave_DataSend(I2C_REGDEF_t *pI2Cx,uint8_t data){
	pI2Cx->DR=data;
}
uint8_t I2C_Slave_ReceiveData(I2C_REGDEF_t *pI2Cx){
	return pI2Cx->DR;
}
















//*****************************************************************************************************************

void I2C_IRQCONFIG(uint8_t IRQNO,uint8_t EnableDisable){
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

void I2C_IRQPriority(uint32_t IRQNumber,uint8_t Priority){
	uint8_t irp=IRQNumber/4;
		uint8_t irp_section=IRQNumber%4;
		uint8_t Shift=(8*irp_section)+4;

		*(NVIC_Priority +(irp*4))|=(Priority<<Shift);
}
