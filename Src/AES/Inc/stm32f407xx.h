/*
 * stm32f407xx.h
 *
 *  Created on: Jan 22, 2023
 *      Author: AKhal
 */


#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISRER0     				((__vo uint32_t*)0xE000E100)
#define NVIC_ISRER1     				((__vo uint32_t*)0xE000E104)
#define NVIC_ISRER2     				((__vo uint32_t*)0xE000E108)
#define NVIC_ISRER3    		    		((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0                       ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1                       ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2                       ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3                       ((__vo uint32_t*)0xE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 				((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASHM_BASEADDR      			0x08000000U
#define SRAM1_BASEADDR      			0x20000000U
#define SRAM2_BASEADDR      			0x2001C000U
#define ROM_BASEADDR	    			0x1FFF0000U
#define SRAM                    		SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 	    		0x40000000U
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000U
#define AHB1PERIPH_BASEADDR				0x40020000U
#define AHB2PERIPH_BASEADDR				0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */


#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)
#define FLASH_BASEADDR                  (AHB1PERIPH_BASEADDR + 0x3C00)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3C00)


#define USART2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR                  (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR                  (APB1PERIPH_BASEADDR + 0x5000)

#define PWR_BASEADDR                    (APB1PERIPH_BASEADDR + 0x7000)

#define TIM2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR                   (APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR                   (APB1PERIPH_BASEADDR + 0x1400)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR                 (APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                 (APB2PERIPH_BASEADDR + 0x1400)

#define ADC1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x2000)
#define ADC2_BASEADDR                   (APB2PERIPH_BASEADDR + 0x2100)
#define ADC3_BASEADDR                   (APB2PERIPH_BASEADDR + 0x2200)
#define ADC123_COMMON_REG               (APB2PERIPH_BASEADDR + 0x2000)

#define DAC_BASEADDR                    (APB1PERIPH_BASEADDR + 0x7400)


/**********************************peripheral register definition structures **********************************/


typedef struct{

	__vo uint32_t MODER;                   //this register used to determine operation mode of a certain pin
	__vo uint32_t OTYPER;				   //this register used to determine output type of certain pin whether it push-pull or open-drain
	__vo uint32_t OSPEEDR;				   //this register used to configure I/O output speed
	__vo uint32_t PUPDR;				   //this register used to configure I/O pull-up or pull-down
	__vo uint32_t IDR;					   //this register contain the input value of I/O port
	__vo uint32_t ODR;                     //this register contain the output value of port
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;					   //this register is used to lock the configuration of the port bits
	__vo uint32_t AFR[2];				   //tis registers used to determine alternate functionality of I/O pins

}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC
 */
typedef struct{

	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t Reserved1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t Reserved2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t Reserved3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t Reserved4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t Reserved5;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	__vo uint32_t Reserved6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t Reserved7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;

}RCC_RegDef_t;

typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWEIR;
	__vo uint32_t PR;
}EXTI_regDef_t;


typedef struct{

	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	__vo uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */

typedef struct{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;


}SPI_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */

typedef struct{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;



/*
 * peripheral register definition structure for USART
 */
typedef struct{

	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BBR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;



/*
 * peripheral register definition structure for PWR
 */
typedef struct{

	__vo uint32_t CR;
	__vo uint32_t CSR;
}PWR_RegDef_t;

/*
 * peripheral register definition structure for Flash Interface Register
 */
typedef struct{

	__vo uint32_t ACR;
	__vo uint32_t KEYR;
	__vo uint32_t OPTKEYR;
	__vo uint32_t SR;
	__vo uint32_t CR;
	__vo uint32_t OPTCR;
	__vo uint32_t OPTCR1;
}FLASH_RegDef_t;

/*
 * peripheral register definition structure for Timers Register
 */
typedef struct{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1;
	__vo uint32_t CCMR2;
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t reserved1;
	__vo uint32_t CCR1;
	__vo uint32_t CCR2;
	__vo uint32_t CCR3;
	__vo uint32_t CCR4;
	__vo uint32_t reserved2;
	__vo uint32_t DCR;
	__vo uint32_t DMAR;
	__vo uint32_t OR;
}TIMER_RegDef_t;



/*
 * peripheral register definition structure for ADCx Register
 */
typedef struct{

	__vo uint32_t SR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMPR1;
	__vo uint32_t SMPR2;
	__vo uint32_t JOFR1;
	__vo uint32_t JOFR2;
	__vo uint32_t JOFR3;
	__vo uint32_t JOFR4;
	__vo uint32_t HTR;
	__vo uint32_t LTR;
	__vo uint32_t SQR1;
	__vo uint32_t SQR2;
	__vo uint32_t SQR3;
	__vo uint32_t JSQR;
	__vo uint32_t JDR1;
	__vo uint32_t JDR2;
	__vo uint32_t JDR3;
	__vo uint32_t JDR4;
	__vo uint32_t DR;
	__vo uint32_t CSR;
	__vo uint32_t CCR;
	__vo uint32_t CDR;
}ADC_RegDef_t;




/*
 * peripheral register definition structure for DAC Register
 */
typedef struct{

	__vo uint32_t CR;
	__vo uint32_t SWTRICR;
	__vo uint32_t DHR12R1;
	__vo uint32_t DHR12L1;
	__vo uint32_t DHR8R1;
	__vo uint32_t DHR12R2;
	__vo uint32_t DHR12L2;
	__vo uint32_t DHR8R2;
	__vo uint32_t DHR12RD;
	__vo uint32_t DHR12LD;
	__vo uint32_t DHR8RD;
	__vo uint32_t DOR1;
	__vo uint32_t DOR2;
	__vo uint32_t SR;
}DAC_Reg_t;


/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */


#define GPIOA                 ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                 ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                 ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                 ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                 ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                 ((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC                   ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                  ((EXTI_regDef_t*)EXTI_BASEADDR)
#define SYSCFG				  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1                  ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                  ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                  ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1                   ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                   ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                   ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1                 ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2                 ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3                 ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4                  ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5                  ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6                 ((USART_RegDef_t*)USART6_BASEADDR)

#define PWR                    ((PWR_RegDef_t*)PWR_BASEADDR)
#define FLASH                  ((FLASH_RegDef_t*)FLASH_BASEADDR)

#define TIMER2                 ((TIMER_RegDef_t*)TIM2_BASEADDR)
#define TIMER3                 ((TIMER_RegDef_t*)TIM3_BASEADDR)
#define TIMER4                 ((TIMER_RegDef_t*)TIM4_BASEADDR)
#define TIMER5                 ((TIMER_RegDef_t*)TIM5_BASEADDR)
#define TIMER6                 ((TIMER_RegDef_t*)TIM6_BASEADDR)
#define TIMER7                 ((TIMER_RegDef_t*)TIM7_BASEADDR)

#define ADC1                   ((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2                   ((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3                   ((ADC_RegDef_t*)ADC3_BASEADDR)

#define DAC                    ((DAC_Reg_t*)DAC_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		  (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		  (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		  (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		  (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		  (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		  (RCC->AHB1ENR |= (1 << 5))


/*
 * Clock Disnable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		  (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		  (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		  (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		  (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		  (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		  (RCC->AHB1ENR &= ~(1 << 5))


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() 		(RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 14))

/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))


/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()           (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()           (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()           (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()           (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()           (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()           (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()         (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()         (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()         (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()          (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()          (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()         (RCC->APB2ENR |= (1 << 5))


/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()         (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()         (RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Enable Macros for PWR peripheral
 */
#define PWR_PCLK_EN()            (RCC->APB1ENR |= (1 << 28))

/*
 * Clock Enable Macros for Timerx peripheral
 */
#define TIM2_PCLK_EN()            (RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()            (RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()            (RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()            (RCC->APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN()            (RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN()            (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Disable Macros for Timerx peripheral
 */
#define TIM2_PCLK_DI()            (RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()            (RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()            (RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()            (RCC->APB1ENR &= ~(1 << 3))
#define TIM6_PCLK_DI()            (RCC->APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DI()            (RCC->APB1ENR &= ~(1 << 5))



/*
 * Clock Enable Macros for ADCx peripheral
 */
#define ADC1_PCLK_EN()            (RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN()            (RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN()            (RCC->APB2ENR |= (1 << 10))


/*
 * Clock Disable Macros for ADCx peripheral
 */
#define ADC1_PCLK_DI()            (RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DI()            (RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DI()            (RCC->APB2ENR &= ~(1 << 10))


/*
 * Clock Enable Macros for DAC peripheral
 */
#define DAC_PCLK_EN()             (RCC->APB1ENR |= (1 << 29))



/*
 * Clock Disable Macros for DAC peripheral
 */
#define DAC_PCLK_DI()             (RCC->APB1ENR &= ~(1 << 29))



/*
 *  Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 0)) ; (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 1)) ; (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 2)) ; (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 3)) ; (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 4)) ; (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()       do{(RCC->AHB1RSTR |= (1 << 5)) ; (RCC->AHB1RSTR &= ~(1 << 5));}while(0)

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()       do{(RCC->APB2RSTR |= (1 << 12)) ; (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()       do{(RCC->APB1RSTR |= (1 << 14)) ; (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()       do{(RCC->APB1RSTR |= (1 << 15)) ; (RCC->APB1RSTR &= ~(1 << 15));}while(0)


/*
 *  Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()       do{(RCC->APB1RSTR |= (1 << 21)) ; (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()       do{(RCC->APB1RSTR |= (1 << 22)) ; (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()       do{(RCC->APB1RSTR |= (1 << 23)) ; (RCC->APB1RSTR &= ~(1 << 23));}while(0)


/*
 *  Macros to reset USARTx peripherals
 */
#define USART1_REG_RESET()     do{(RCC->APB2RSTR |= (1 << 4)) ; (RCC->APB2RSTR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()     do{(RCC->APB1RSTR |= (1 << 17)) ; (RCC->APB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()     do{(RCC->APB1RSTR |= (1 << 18)) ; (RCC->APB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()      do{(RCC->APB1RSTR |= (1 << 19)) ; (RCC->APB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()      do{(RCC->APB1RSTR |= (1 << 20)) ; (RCC->APB1RSTR &= ~(1 << 20));}while(0)
#define USART6_REG_RESET()     do{(RCC->APB2RSTR |= (1 << 5)) ; (RCC->APB2RSTR &= ~(1 << 5));}while(0)

/*
 *  Macros to reset TIMERx peripherals
 */
#define TIM2_REG_RESET()        do{(RCC->APB1RSTR |= (1 << 0)) ; (RCC->APB1RSTR &= ~(1 << 0));}while(0)
#define TIM3_REG_RESET()        do{(RCC->APB1RSTR |= (1 << 1)) ; (RCC->APB1RSTR &= ~(1 << 1));}while(0)
#define TIM4_REG_RESET()        do{(RCC->APB1RSTR |= (1 << 2)) ; (RCC->APB1RSTR &= ~(1 << 2));}while(0)
#define TIM5_REG_RESET()        do{(RCC->APB1RSTR |= (1 << 3)) ; (RCC->APB1RSTR &= ~(1 << 3));}while(0)
#define TIM6_REG_RESET()        do{(RCC->APB1RSTR |= (1 << 4)) ; (RCC->APB1RSTR &= ~(1 << 4));}while(0)
#define TIM7_REG_RESET()        do{(RCC->APB1RSTR |= (1 << 5)) ; (RCC->APB1RSTR &= ~(1 << 5));}while(0)



/*
 *  Macros to reset ADCx peripherals
 */
#define ADC_REG_RESET()         do{(RCC->APB2RSTR |= (1 << 8)) ; (RCC->APB2RSTR &= ~(1 << 8));}while(0)


/*
 *  Macros to reset DAC peripherals
 */
#define DAC_REG_RESET()         do{(RCC->APB1RSTR |= (1 << 29)) ; (RCC->APB1RSTR &= ~(1 << 29));}while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)  (( x == GPIOA) ? 0 :\
									( x == GPIOB) ? 1 :\
								    ( x == GPIOC) ? 2 :\
								    ( x == GPIOD) ? 3 :\
								    ( x == GPIOE) ? 4 :\
								    ( x == GPIOF) ? 5 :0)



/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71
#define IRQ_NO_TIM2         28
#define IRQ_NO_TIM3         29
#define IRQ_NO_TIM4         30



/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


//some generic macros

#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET        	SET
#define GPIO_PIN_RESET     	 	RESET
#define FLAG_RESET         		RESET
#define FLAG_SET 				SET





/*
 *  Bit positions of RCC_CFGR register
 */
#define RCC_CFGR_MCO1_POS         21
#define RCC_CFGR_MCO1             (0x3 << RCC_CFGR_MCO1_POS)

#define RCC_CFGR_MCO2_POS         30
#define RCC_CFGR_MCO2             (0x3 << RCC_CFGR_MCO2_POS)

#define RCC_CFGR_MCO1PRE_POS      24
#define RCC_CFGR_MCO1PRE          (0x7 << RCC_CFGR_MCO1PRE_POS)

#define RCC_CFGR_MCO2PRE_POS      27
#define RCC_CFGR_MCO2PRE          (0x7 << RCC_CFGR_MCO2PRE_POS)

#define RCC_CFGR_SW_POS           0
#define RCC_CFGR_SW               (0x3 << RCC_CFGR_SW_POS)

#define RCC_CFGR_SWS_POS           2
#define RCC_CFGR_SWS               (0x3 << RCC_CFGR_SWS_POS)

#define RCC_CFGR_HPRE_POS          4
#define RCC_CFGR_PPRE1_POS         10
#define RCC_CFGR_PPRE2_POS         13


/*
 *  Bit positions of RCC_PLLCFGR register
 */
#define RCC_PLLCFGR_PLLSRC_POS     22
#define RCC_PLLCFGR_PLLSRC         (1 << RCC_PLLCFGR_PLLSRC_POS)

#define RCC_PLLCFGR_M_POS          0
#define RCC_PLLCFGR_M              (0x3F << RCC_PLLCFGR_M_POS)

#define RCC_PLLCFGR_N_POS          6
#define RCC_PLLCFGR_N              (0x1FF << RCC_PLLCFGR_N_POS)

#define RCC_PLLCFGR_Q_POS          24
#define RCC_PLLCFGR_Q              (0xF << RCC_PLLCFGR_Q_POS)

#define RCC_PLLCFGR_P_POS          16
#define RCC_PLLCFGR_P              (0x3 << RCC_PLLCFGR_P_POS)

/*
 * Bit positions of RCC_CR register
 */
#define RCC_CR_HSION              0
#define RCC_CR_HSIRDY             1
#define RCC_CR_HSEON              16
#define RCC_CR_HSERDY             17
#define RCC_CR_PLLON              24
#define RCC_CR_PLLRDY             25


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA             0
#define SPI_CR1_CPOL             1
#define SPI_CR1_MSTR             2
#define SPI_CR1_BR               3    //Baud rate
#define SPI_CR1_SPE              6
#define SPI_CR1_LSBFIRST         7    //0:MSB transmitted first , 1: LSB transmitted first
#define SPI_CR1_SSI              8    //This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the NSS pin and the IO value of the NSS pin is ignored.
#define SPI_CR1_SSM              9
#define SPI_CR1_RXONLY           10
#define SPI_CR1_DFF              11
#define SPI_CR1_CRCNEXT          12
#define SPI_CR1_CRCEN            13
#define SPI_CR1_BIDIOE           14   //This bit combined with the BIDImode bit selects the direction of transfer in bidirectional mode
#define SPI_CR1_BIDIMODE         15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN           0
#define SPI_CR2_TXDMAEN           1
#define SPI_CR2_SSOE              2
#define SPI_CR2_FRF               4
#define SPI_CR2_ERRIE             5
#define SPI_CR2_RXNEIE            6
#define SPI_CR2_TXEIE             7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE               0
#define SPI_SR_TXE                1
#define SPI_SR_CHSIDE             2
#define SPI_SR_UDR                3
#define SPI_SR_CRCERR             4
#define SPI_SR_MODF               5
#define SPI_SR_OVR                6
#define SPI_SR_BSY                7
#define SPI_SR_FRE                8



/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE                0
#define I2C_CR1_NOSTERETCH        7
#define I2C_CR1_START             8
#define I2C_CR1_STOP              9
#define I2C_CR1_ACK               10
#define I2C_CR1_SWRST             15


/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ               0
#define I2C_CR2_ITERREN            8
#define I2C_CR2_ITEVTEN            9
#define I2C_CR2_ITBUFEN           10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0               0
#define I2C_OAR1_ADD71              1
#define I2C_OAR1_ADD98              8
#define I2C_OAR1_ADDMODE            15


/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB                   0
#define I2C_SR1_ADDR                 1
#define I2C_SR1_BTF                  2
#define I2C_SR1_ADD10                3
#define I2C_SR1_STOPF                4
#define I2C_SR1_RXNE                 6
#define I2C_SR1_TXE                  7
#define I2C_SR1_BERR                 8
#define I2C_SR1_ARLO                 9
#define I2C_SR1_AF                   10
#define I2C_SR1_OVR                  11
#define I2C_SR1_TIMEOUT              14



/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL                  0
#define I2C_SR2_BUSY                 1
#define I2C_SR2_TRA                  2
#define I2C_SR2_GENCALL              4
#define I2C_SR2_DUALF                7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR                  0
#define I2C_CCR_DUTY                 14
#define I2C_CCR_FS                   15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/


/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK                0   //SEND BREAK : This bit set is used to send break characters. It can be set and cleared by software. It shouldbe set by software, and will be reset by hardware during the stop bit of break.
#define USART_CR1_RWU                1   //RECEIVER WAKEUP : This bit determines if the USART is in mute mode or not
#define USART_CR1_RE                 2   //RECEIVER ENABLE : This bit enables the receiver
#define USART_CR1_TE                 3   //TRANSMITTER ENABLE : This bit enables the transmitter
#define USART_CR1_IDLEIE             4   //IDLE INTERRUPT ENABLE
#define USART_CR1_RXNEIE             5   //RXNE INTERRUPT ENABLE
#define USART_CR1_TCIE               6   //TRANSMISSION CONTROL INTERRUPT ENABLE
#define USART_CR1_TXEIE              7   //TXEIE INTERRUPT ENABLE
#define USART_CR1_PEIE               8   //PE INTERRUPT ENABLE
#define USART_CR1_PS                 9   //PARITY SELECTION : This bit selects the odd or even parity
#define USART_CR1_PCE                10  //PARITY CONTROL BIT : This bit selects the hardware parity control (generation and detection).
#define USART_CR1_WAKE               11  //WAKEUP METHOD : This bit determines the USART wakeup method.
#define USART_CR1_M                  12  //WORD LENGTH : This bit determines the word length.
#define USART_CR1_UE                 13  //USART ENABLE
#define USART_CR1_OVER8              15  //OVERSAMPLING MODE : it can be either 16 or 8 times the baud rate clock

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR2_ADD                0
#define USART_CR2_LBDL               5
#define USART_CR2_LBDIE              6
#define USART_CR2_LBCL               8
#define USART_CR2_CPHA               9
#define USART_CR2_CPOL               10
#define USART_CR2_CLKEN              11
#define USART_CR2_STOP               12
#define USART_CR2_LINEN              14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11


/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


/*
 *  Bit position definitons of PWR_CR register
 */
#define PWR_CR_VOS_POS                  14
#define PWR_CR_VOS                      (0x1 << PWR_CR_VOS_POS)


/*
 *  Bit position definitons of FLASH_ACR register
 */
#define FLASH_ACK_LATENCY               0
#define FLASH_ACK_DCEN                  10



#define FLASH_ACR_LATENCY_0WS           0x00000000U
#define FLASH_ACR_LATENCY_1WS           0x00000001U
#define FLASH_ACR_LATENCY_2WS           0x00000002U
#define FLASH_ACR_LATENCY_3WS           0x00000003U
#define FLASH_ACR_LATENCY_4WS           0x00000004U
#define FLASH_ACR_LATENCY_5WS           0x00000005U
#define FLASH_ACR_LATENCY_6WS           0x00000006U
#define FLASH_ACR_LATENCY_7WS           0x00000007U


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 *  Bit position definitons of Timer_CR1 register
 */
#define TIMER_CR1_CEN                   0
#define TIMER_CR1_UDIS                  1
#define TIMER_CR1_URS                   2
#define TIMER_CR1_OPM                   3
#define TIMER_CR1_DIR                   4
#define TIMER_CR1_CMS                   5
#define TIMER_CR1_ARPE                  7
#define TIMER_CR1_CKD                   8

/*
 *  Bit position definitons of Timer_EGR register
 */
#define TIMER_EGR_UG                    0

/*
 *  Bit position definitons of Timer_EGR register
 */
#define TIMER_DIER_UIE                   0


/*
 *  Bit position definitons of Timer_EGR register
 */
#define TIMER_SR_UIF                     0


/******************************************************************************************
 *Bit position definitions of ADC peripheral
 ******************************************************************************************/

/*
 * Bit posistion defination of ADC_SR register
 */
#define ADC_SR_AWD                        0
#define ADC_SR_EOC                        1
#define ADC_SR_JEOC                       2
#define ADC_SR_JSTRT                      3
#define ADC_SR_STRT                       4
#define ADC_SR_OVR                        5


/*
 * Bit posistion defination of ADC_CR1 register
 */
#define ADC_CR1_AWDCH                      0
#define ADC_CR1_EOCIE                      5
#define ADC_CR1_AWDIE                      6
#define ADC_CR1_JEOCIE                     7
#define ADC_CR1_SCAN                       8
#define ADC_CR1_AWDSGL                     9
#define ADC_CR1_JAUTO                      10
#define ADC_CR1_DISCEN                     11
#define ADC_CR1_JDISCEN                    12
#define ADC_CR1_DISCNUM                    13
#define ADC_CR1_JAWDEN                     22
#define ADC_CR1_AWDEN                      23
#define ADC_CR1_RES                        24
#define ADC_CR1_OVRIE                      26


/*
 * Bit posistion defination of ADC_CR2 register
 */
#define ADC_CR2_ADON                       0
#define ADC_CR2_CONT                       1
#define ADC_CR2_DMA                        8
#define ADC_CR2_DDS                        9
#define ADC_CR2_EOCS                       10
#define ADC_CR2_ALIGN                      11
#define ADC_CR2_SWSTART                    30

/*
 * Bit posistion defination of ADC_SQRx registers
 */

//SQ1 register
#define ADC_SQ13                            0
#define ADC_SQ14                            5
#define ADC_SQ15                            10
#define ADC_SQ16                            15
#define ADC_SQL                             20

//SQ2 register
#define ADC_SQ7                             0
#define ADC_SQ8                             5
#define ADC_SQ9                             10
#define ADC_SQ10                            15
#define ADC_SQ11                            20
#define ADC_SQ12                            25


//SQ3 register
#define ADC_SQ1                             0
#define ADC_SQ2                             5
#define ADC_SQ3                             10
#define ADC_SQ4                             15
#define ADC_SQ5                             20
#define ADC_SQ6                             25


/******************************************************************************************
 *Bit position definitions of DAC peripheral
 ******************************************************************************************/

/*
 * Bit posistion defination of DAC_CR register
 */
#define DAC_CR_EN1                           0
#define DAC_CR_BOFf1                         1
#define DAC_CR_TEN1                          2
#define DAC_CR_TSEL1                         3
#define DAC_CR_WAVE1                         6
#define DAC_CR_MAMP1                         8
#define DAC_CR_DMAEN1                        12
#define DAC_CR_DMAUDRIE1                     13
#define DAC_CR_EN2                           16
#define DAC_CR_BOFF2                         17
#define DAC_CR_TEN2                          18
#define DAC_CR_TSEL2                         19
#define DAC_CR_WAVE2                         22
#define DAC_CR_MAMP2                         24
#define DAC_CR_DMAEN2                        28
#define DAC_CR_DMAUDRIE2                     29



#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_usart_driver.h"
#endif /* INC_STM3F407XX_H_ */

