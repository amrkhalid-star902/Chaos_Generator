/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 22, 2023
 *      Author: AKhal
 */

#ifndef STM32F407XX_GPIO_DRIVER_H_
#define STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */

typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t PinMode;
	uint8_t PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */

#define GPIO_MODE_IN                0
#define GPIO_MODE_OUT               1
#define GPIO_MODE_ALTFN             2
#define GPIO_MODE_ANALOG            3
#define GPIO_MODE_IT_FT             4 //integer triggered on falling edge
#define GPIO_MODE_IT_RT             5 //integer triggered on rising edge
#define GPIO_MODE_IT_RFT            6 //integer triggered on both edges

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP     		0 //push-pull
#define GPIO_OP_TYPE_OD   			1 //open drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPOI_SPEED_HIGH				3



/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/*
 * Peripheral Clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGpiox , uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_InitDirect(GPIO_RegDef_t *pGPIOx , uint8_t pinNumber , uint8_t mode , uint8_t speed , uint8_t outputType , uint8_t PuorPdr , uint8_t altfunction );
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber , uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,  uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* STM32F407XX_GPIO_DRIVER_H_ */



