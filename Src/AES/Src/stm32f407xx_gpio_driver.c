/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jan 22, 2023
 *      Author: AKhal
 */

#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGpiox , uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if( pGpiox == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGpiox == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGpiox == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGpiox == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGpiox == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGpiox == GPIOF){
			GPIOF_PCLK_EN();
		}
	}else{
		if( pGpiox == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGpiox == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGpiox == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGpiox == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGpiox == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGpiox == GPIOF){
			GPIOF_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -
 *
 * @param[in]         - gpiohandler
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp=0;

	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.PinMode <= GPIO_MODE_ANALOG){
		//Non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}else{
		//Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT){
			//1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}else if(pGPIOHandle->GPIO_PinConfig.PinMode ==GPIO_MODE_IT_RFT){
			//1. configure both the FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );

		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR

		uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
		uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		//3. enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

	}

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_ALTFN){
		//configure the alternate function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2));//clearing pervious values
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));//clearing pervious values

	}

}

void GPIO_InitDirect(GPIO_RegDef_t *pGPIOx , uint8_t pinNumber , uint8_t mode , uint8_t speed , uint8_t outputType , uint8_t PuorPdr , uint8_t altfunction ){

	GPIO_Handle_t GPIOHandler;
	GPIOHandler.pGPIOx = pGPIOx;
	GPIOHandler.GPIO_PinConfig.GPIO_PinAltFunMode = altfunction;
	GPIOHandler.GPIO_PinConfig.GPIO_PinNumber = pinNumber;
	GPIOHandler.GPIO_PinConfig.GPIO_PinOPType = outputType;
	GPIOHandler.GPIO_PinConfig.GPIO_PinPuPdControl = PuorPdr;
	GPIOHandler.GPIO_PinConfig.PinMode = mode;
	GPIOHandler.GPIO_PinConfig.PinSpeed = speed;

	GPIO_Init(&GPIOHandler);
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -gpio port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             -
 *
 * @param[in]         -gpio port
 * @param[in]         -pinNumber
 * @param[in]         -
 *
 * @return            -   0 or 1
 *
 * @Note              -
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber){

	uint8_t value = (uint8_t)(pGPIOx->IDR >> PinNumber)&0x00000001;
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             -
 *
 * @param[in]         - gpio port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - the input value to gpio port
 *
 * @Note              -
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             -
 *
 * @param[in]         - 0 or 1
 * @param[in]         - gpio port
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx , uint8_t PinNumber , uint8_t value){

	if(value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             -
 *
 * @param[in]         - value to be written
 * @param[in]         - gpio port
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx ,  uint16_t value){
	pGPIOx->ODR |= (value);

}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             -
 *
 * @param[in]         - pinNumber
 * @param[in]         - gpio port
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         - IRQ number
 * @param[in]         - 0 or 1
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(IRQNumber <= 31){
			*NVIC_ISRER0 |= (1 << IRQNumber);
		}else if (IRQNumber > 31 && IRQNumber <= 63){
			*NVIC_ISRER1 |= (1 << (IRQNumber %32));

		}else if(IRQNumber > 63 && IRQNumber <= 95){
			*NVIC_ISRER2 |= (1 << (IRQNumber %64));

		}else if(IRQNumber > 95 && IRQNumber <= 127){
			*NVIC_ISRER3 |= (1 << (IRQNumber %96));

		}
	}else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber <= 63){
			*NVIC_ICER1 |= (1 << (IRQNumber %32));

		}else if(IRQNumber > 63 && IRQNumber <= 95){
			*NVIC_ICER2 |= (1 << (IRQNumber %64));

		}else if(IRQNumber > 95 && IRQNumber <= 127){
			*NVIC_ICER3 |= (1 << (IRQNumber %96));

		}
	}
}


/*********************************************************************
 * @fn      		  - IRQPriorityConfig
 *
 * @brief             -
 *
 * @param[in]         - IRQNumber
 * @param[in]         - Priority
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber , uint32_t IRQPriority){
	//1. first lets find out the ipriority(ipr) register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         - pinNumber
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -
 */

void GPIO_IRQHandling(uint8_t PinNumber){

	//clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){

		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
