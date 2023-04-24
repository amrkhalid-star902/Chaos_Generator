/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: Jan 25, 2023
 *      Author: AKhal
 */

#ifndef STM32F407XX_RCC_DRIVER_H_
#define STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"


/*************************Macro defination************************/
/*
 *  MCO1 and MCO2 are external clock output pins
 */
#define RCC_MCO1_HSI                      0
#define RCC_MCO1_LSE                      1
#define RCC_MCO1_HSE                      2
#define RCC_MCO1_PLL                      3


#define RCC_MCO2_SYSCLK                    0
#define RCC_MCO2_PLLI2S                    1
#define RCC_MCO2_HSE                       2
#define RCC_MCO2_PLL                       3

#define MCO_DIV2                           4
#define MCO_DIV3                           5
#define MCO_DIV4                           6
#define MCO_DIV5                           7


#define APB_DIV2                           4
#define APB_DIV4                           5
#define APB_DIV8                           6
#define APB_DIV16                          7


#define AHB_DIV2                           8
#define AHB_DIV4                           9
#define AHB_DIV8                           10
#define AHB_DIV16                          11
#define AHB_DIV64                          12
#define AHB_DIV128                         13
#define AHB_DIV256                         14
#define AHB_DIV512                         15


/*
 * Clock Source Selection
 */
#define RCC_SYSCLK_HSI                      0
#define RCC_SYSCLK_HSE                      1
#define RCC_SYSCLK_PLL                      2

#define RCC_PLL_84_VCO_INPUT_DIV            8
#define RCC_PLL_84_VCO_OUTPUT_MUL           84
#define RCC_PLL_84_PLL_OUTPUT_DIV           0
#define RCC_PLL_84_MAIN_PLL_DIV             7

//setting configuration of output clock pins
void RCC_MCO1_Config(uint8_t option , uint8_t prescaler);
void RCC_MCO2_Config(uint8_t option , uint8_t prescaler);

//set HSE as system clock
void RCC_Set_SYSCLK_HSE(void);

//set HSI as system clock
void RCC_Set_SYSCLK_HSI(void);

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

//This returns the PLL clock value
uint32_t  RCC_GetPLLOutputClock(void);

void RCC_Set_SYSCLK_PLL_84_MHz(void);


#endif /* STM32F407XX_RCC_DRIVER_H_ */
