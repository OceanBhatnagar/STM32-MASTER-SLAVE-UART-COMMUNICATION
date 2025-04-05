#ifndef INC_STM_H_
#define INC_STM_H_

#include<stddef.h>
#include<stdint.h>
#define __weak __attribute__((weak))

//Base Address of NVIC

#define NVIC_ISER0  ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1  ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2  ((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3  ((volatile uint32_t*)0xE000E10c)

#define NVIC_ICER0   ((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1  ((volatile uint32_t*)0XE000E184)
#define NVIC_ICER2   ((volatile uint32_t*)0XE000E188)
#define NVIC_ICER3   ((volatile uint32_t*)0XE000E18c)

#define NVIC_Priority   ((volatile uint32_t*)0xE000E400)
//Base Address of Memory

#define FLASH_BASEADDR  0x08000000U
#define SRAM_BASEADDR   0x20000000U
#define ROM_BASEADDR    0x1FFF0000U

//Base Address of Bus

#define PERIPHERAL_BASEADDR  0x40000000U
#define APB1_PERIPHERAL_BASEADDR 0x40000000U
#define AHB1_PERIPHERAL_BASEADDR 0x40020000U
#define APB2_PERIPHERAL_BASEADDR  0x40010000U

//Base Address of AHB1 Peripherals

#define GPIOA_BASEADDR  0x40020000U
#define GPIOB_BASEADDR  0x40020400U
#define GPIOC_BASEADDR  0x40020800U
#define GPIOH_BASEADDR  0x40021C00U
#define RCC_BASEADDR    0x40023800
//Base Address of APB1 Peripherals

#define I2C1_BASEADDR   0x40005400U
#define I2C2_BASEADDR   0x40005800U
#define I2C4_BASEADDR   0x40006000U
#define SPI2_BASEADDR   0x40003800U
#define USART2_BASEADDR 0x40004400U

//Base Address of APB2 Peripherals

#define USART1_BASEADDR  0x40011000U
#define USART6_BASEADDR  0x40011400U
#define SPI1_BASEADDR    0x40013000U
#define SPI5_BASEADDR    0x40015000U
#define EXTI_BASEADDR    0x40013C00U
#define SYSCFG_BASEADDR  0x40013800U




//Peripheral Register Defination
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDER;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
} GPIO_REGDEF_t;

#define GPIOA ((GPIO_REGDEF_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_REGDEF_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_REGDEF_t*)GPIOC_BASEADDR)
#define GPIOH ((GPIO_REGDEF_t*)GPIOH_BASEADDR)

//**********************************************************************************


//EXTI PERIPHERAL
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_REGDEF_t;

#define EXTI ((EXTI_REGDEF_t*)EXTI_BASEADDR)

//SYSCFG PERIPHERAL
typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[3];
	volatile uint32_t CFGR2;
	volatile uint32_t CMPCR;
	volatile uint32_t CFGR;
} SYSCFG_REGDEF_t;

#define SYSCFG ((SYSCFG_REGDEF_t*)SYSCFG_BASEADDR)

//IRQ NO.

#define IRQ_EXTI0 6
#define IRQ_EXTI1 7
#define IRQ_EXTI2 8
#define IRQ_EXTI3 9
#define IRQ_EXTI4 10
#define IRQ_EXTI9_5 23
#define IRQ_EXTI15_10 40

//This macro returns a code( between 0 to 7) for a given GPIO base address(x)
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
								        (x == GPIOH)?7:0)
//*********************************************************************************************

//I2C PERIPHERAL DEFINATIONS

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;     //ADDRESS
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
} I2C_REGDEF_t;

#define I2C1_R ((I2C_REGDEF_t*)I2C1_BASEADDR)
#define I2C2 ((I2C_REGDEF_t*)I2C2_BASEADDR)
#define I2C4 ((I2C_REGDEF_t*)I2C4_BASEADDR)


//*********************************************************************************************

//USART PERIPHERAL DEFINATION


typedef struct
{
	volatile uint32_t USART_SR;
	volatile uint32_t USART_DR;
	volatile uint32_t USART_BRR;     //ADDRESS
	volatile uint32_t USART_CR1;
	volatile uint32_t USART_CR2;
	volatile uint32_t USART_CR3;
	volatile uint32_t USART_GTPR;
}USART_REGDEF_t;

#define USART1 ((USART_REGDEF_t*)USART1_BASEADDR)
#define USART2 ((USART_REGDEF_t*)USART2_BASEADDR)
#define USART6 ((USART_REGDEF_t*)USART6_BASEADDR)

#include<usart_driver.h>
#include<interrupt_driver.h>
#include<i2c_driver.h>
#endif /* INC_STM32F410RB_H_ */
