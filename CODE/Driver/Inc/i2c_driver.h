/*
 * i2c_driver.h
 *
 *  Created on: Feb 10, 2025
 *      Author: Ocean
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

#include<stdint.h>
#include <stm.h>

//I2C PIN CONFIGURATION

typedef struct{
	uint32_t SCLSpeed;
	uint8_t DeviceAddress;
	uint8_t AckControl;
	uint8_t FM_DutyCycle;
}I2C_Config_t;

typedef struct{
	I2C_REGDEF_t *pI2Cx;
	I2C_Config_t  I2CConfig;
}I2C_Handle_t;

#define I2C_SCL_SPEED_NORMAL  100000
#define I2C_SCL_SPEED_FAST4K  400000
#define I2C_SCL_SPEED_FAST2K  200000

#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

#define I2C_DUTYCYCLE_2 0
#define I2C_DUTYCYCLE_16_9 1

//API PROTOTYPE

//Initialization
void I2C_init(I2C_Handle_t *I2CHandle);

//I2C PERIPHERAL ENABLE
void I2C_PeripheralControl(I2C_REGDEF_t *pI2Cx,uint8_t EnorDi);

// DATA SEND AND RECEIVE MASTER MODE
void I2C_Master_DataSend(I2C_Handle_t *I2CHandle,uint8_t *TxBuffer,uint32_t Len,uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2Chandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr);

//DATA SEND AND RECEIVE SLAVE MODE
void I2C_Slave_DataSend(I2C_REGDEF_t *pI2Cx,uint8_t data);
uint8_t I2C_Slave_ReceiveData(I2C_REGDEF_t *pI2Cx);

//ACKING CONTROL
void I2C_ACK_CONTROL(I2C_REGDEF_t *pI2Cx,uint8_t EnorDi);
//INTERRUPT
void I2C_IRQCONFIG(uint8_t IRQNO,uint8_t EnableDisable);
void I2C_IRQPriority(uint32_t IRQNumber,uint8_t Priority);

uint8_t I2C_GETFLAG(I2C_REGDEF_t *pI2Cx,uint32_t FlagName);
#define FLAG_RESET 0
#define FLAG_SET   1
//I2C STATUS REGISTER

#define I2C_TXE_FLAG  (1<<7)
#define I2C_RXE_FLAG  (1<<6)
#define I2C_SB_FLAG   (1<<0)
#define I2C_ADDR_FLAG (1<<1)
#define I2C_BTF_FLAG  (1<<2)
#define I2C_STOPF_FLAG (1<<4)
#define I2C_BERR_FLAG (1<<8)
#define I2C_ARLO_FLAG (1<<9)
#define I2C_AF_FLAG   (1<<10)
#define I2C_OVR_FLAG  (1<<11)
#define I2C_PECERR_FLAG  (1<<12)
#define I2C_TIMEOUT_FLAG (1<<14)


#endif /* INC_I2C_DRIVER_H_ */
