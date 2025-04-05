/*
 * interrupt_driver.h
 *
 *  Created on: Jan 26, 2025
 *      Author: Ocean
 */

#ifndef INTERRUPT_DRIVER_H_
#define INTERRUPT_DRIVER_H_


#include<stdint.h>
#include <stm.h>


//GPIO PIN CONFIGURATION

typedef struct{
	uint8_t PinNumber;
	uint8_t PinMode;
	uint8_t PinSpeed;
	uint8_t PinPuPdControl;
	uint8_t PinOPType;
	uint8_t PinAltFunMode;
}GPIO_CONFIG_t;

//GPIO HANDLE
typedef struct{
	GPIO_REGDEF_t *pGPIOx;
	GPIO_CONFIG_t GPIO_CONFIG_t;
}GPIO_HANDLE_t;

//API PROTOTYPE



//GPIO PIN POSSIBLE MODES
#define GPIOMODE_INPUT 0
#define GPIOMODE_OUTPUT 1
#define GPIOMODE_AltFunction 2
#define GPIOMODE_Analog 3
#define GPIOMODE_InterruptFT 4
#define GPIOMODE_InterruptHT 5
#define GPIOMODE_TRIGGER 6

//GPIO OUTPUT TYPE POSSIBLE MODES
#define OUTPUT_PUSHPULL 0
#define OUTPUT_OPENDRAIN 1

//GPIO OUTPUT SPEED
#define SPEED_LOW 0
#define SPEED_MEDIUM 1
#define SPEED_HIGH 2
#define SPEED_VERYHIGH 3

//GPIO PULL UP/DOWN
#define NOPULL 0
#define PULLUP 1
#define PULLDOWN 2



//Initialization and De-initialization
void GPIO_init(GPIO_HANDLE_t *pGPIOHandle);
void GPIO_deinit(GPIO_REGDEF_t *pGPIOx);

//INPUT
uint8_t GPIO_INPUT_PINREAD(GPIO_REGDEF_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_INPUT_PORTREAD(GPIO_REGDEF_t *pGPIOx);

//OUTPUT
void GPIO_OUTPUT_PINWRITE(GPIO_REGDEF_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_OUTPUT_PORTWRITE(GPIO_REGDEF_t *pGPIOx,uint16_t Value);
void GPIO_OUTPUT_TOGGLEPIN(GPIO_REGDEF_t *pGPIOx,uint8_t PinNumber);

//INTERRUPT
void GPIO_IRQCONFIG(uint8_t IRQNO,uint8_t EnableDisable);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriority(uint32_t IRQNumber,uint8_t Priority);

#endif /* INTERRUPT_DRIVER_H_ */
