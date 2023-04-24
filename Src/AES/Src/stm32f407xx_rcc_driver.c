/*
 * stm32f407xx_RCC_driver.c
 *
 *  Created on: Jan 25, 2023
 *      Author: AKhal
 */

#include "stm32f407xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,128,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

void RCC_HSI_Clock_Control(uint8_t EnorDi);
void RCC_HSE_Clock_Control(uint8_t EnorDi);
void RCC_PLL_Clock_Control(uint8_t EnorDi);
void PWR_Set_Scale_Mode(void);
void RCC_set_PLL_84_MHz(void);
void FLASH_Set_Latency(void);
void RCC_Delay(volatile uint32_t delay);


void RCC_MCO1_Config(uint8_t option , uint8_t prescaler){

	//clear the perivous value
	RCC->CFGR &= ~(RCC_CFGR_MCO1);
	//set the sourceclock
	RCC->CFGR |= (option << RCC_CFGR_MCO1_POS);

	//Configure Prescalar
	RCC->CFGR &= ~(RCC_CFGR_MCO1PRE);
	RCC->CFGR |= (prescaler << RCC_CFGR_MCO1PRE_POS);

	//PA8 is used as MCO1
	GPIO_Handle_t RCC_GPIO;
	RCC_GPIO.pGPIOx = GPIOA;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	RCC_GPIO.GPIO_PinConfig.PinMode = GPIO_MODE_ALTFN;
	RCC_GPIO.GPIO_PinConfig.PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&RCC_GPIO);
}


void RCC_MCO2_Config(uint8_t option , uint8_t prescaler){

	//clear the perivous value
	RCC->CFGR &= ~(RCC_CFGR_MCO2);
	//set the sourceclock
	RCC->CFGR |= (option << RCC_CFGR_MCO2_POS);

	//Configure Prescalar
	RCC->CFGR &= ~(RCC_CFGR_MCO2PRE);
	RCC->CFGR |= (prescaler << RCC_CFGR_MCO2PRE_POS);

	//PC9 is used as MCO2
	GPIO_Handle_t RCC_GPIO;
	RCC_GPIO.pGPIOx = GPIOC;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	RCC_GPIO.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	RCC_GPIO.GPIO_PinConfig.PinMode = GPIO_MODE_ALTFN;
	RCC_GPIO.GPIO_PinConfig.PinSpeed = GPIO_SPEED_LOW;
	GPIO_Init(&RCC_GPIO);
}

void RCC_Set_SYSCLK_HSE(void){

	//enable HSE
	RCC_HSE_Clock_Control(ENABLE);

	//select HSE as system clock
	RCC->CFGR &= ~(RCC_CFGR_SW);
	RCC->CFGR |= (RCC_SYSCLK_HSE << RCC_CFGR_SW_POS);

	while(((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) != RCC_SYSCLK_HSE );

	//turn off HSI
	RCC_HSI_Clock_Control(DISABLE);
}

void RCC_Set_SYSCLK_HSI(void){

	//enable HSI
	RCC_HSI_Clock_Control(ENABLE);

	//select HSE as system clock
	RCC->CFGR &= ~(RCC_CFGR_SW);
	RCC->CFGR |= (RCC_SYSCLK_HSI << RCC_CFGR_SW_POS);

	while(((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) != RCC_SYSCLK_HSI );

	//turn off HSE
	RCC_HSE_Clock_Control(DISABLE);
}




void RCC_HSE_Clock_Control(uint8_t EnorDi){

	if(EnorDi == ENABLE){

		RCC->CR |= ( 1 << RCC_CR_HSEON);
		while(!(RCC->CR & (1 << RCC_CR_HSERDY)));

	}else{

		RCC->CR &= ~(1 << RCC_CR_HSEON);
	}
}




void RCC_HSI_Clock_Control(uint8_t EnorDi){


	if(EnorDi == ENABLE){

		RCC->CR |= (1 << RCC_CR_HSION);
		while(!(RCC->CR & (1 << RCC_CR_HSIRDY)));

	}else{

		RCC->CR &= ~(1 << RCC_CR_HSION);
	}
}


void RCC_PLL_Clock_Control(uint8_t EnorDi){

	if(EnorDi == ENABLE){

		RCC->CR |= (1 << RCC_CR_PLLON);
		while(!(RCC->CR & (1 << RCC_CR_PLLRDY)));

	}else{

		RCC->CR &= ~(1 << RCC_CR_PLLON);
	}

}

void RCC_set_PLL_84_MHz (void){

	//Turn on HSE
	RCC_HSE_Clock_Control(ENABLE);

	//select HSE as PLL input
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);

	//diasable pll until configuration is done
	RCC_PLL_Clock_Control(DISABLE);

	//program devision factors to achieve 84MHz
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_M);
	RCC->PLLCFGR |= RCC_PLL_84_VCO_INPUT_DIV << RCC_PLLCFGR_M_POS;

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_N);
	RCC->PLLCFGR |= RCC_PLL_84_VCO_OUTPUT_MUL << RCC_PLLCFGR_N_POS;

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_Q);
	RCC->PLLCFGR |= RCC_PLL_84_MAIN_PLL_DIV << RCC_PLLCFGR_Q_POS;

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_P);
	RCC->PLLCFGR |= RCC_PLL_84_PLL_OUTPUT_DIV<< RCC_PLLCFGR_P_POS;

	//turn on PLL
	RCC_PLL_Clock_Control(ENABLE);
}


void PWR_Set_Scale_Mode(void){

	PWR_PCLK_EN();
	PWR->CR |= (PWR_CR_VOS);
}

void FLASH_Set_Latency(void){

	FLASH->ACR &= ~(0x07 << FLASH_ACK_LATENCY);
	FLASH->ACR |= FLASH_ACR_LATENCY_7WS;

	FLASH->ACR |= FLASH_ACK_DCEN;
}


uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1 , SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000;
	}else if(clksrc == 1){
		SystemClk = 8000000;
	}else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb prescaler

	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	}else{
		ahbp = AHB_PreScaler[temp - 8];
	}

	//for apb1 prescaler
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4){
		apb1p = 1;
	}else{
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp)/apb1p;

	return pclk1;
}









uint32_t RCC_GetPCLK2Value(void){

	uint32_t pclk2 , SystemClk;

	uint8_t clksrc,temp,ahbp,apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){

		SystemClk = 16000000;

	}else if(clksrc == 1){

		SystemClk = 8000000;

	}else if(clksrc == 2){

		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb prescaler

	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){

		ahbp = 1;

	}else{

		ahbp = AHB_PreScaler[temp - 8];
	}

	//for apb1 prescaler
	temp = ((RCC->CFGR >> 13) & 0x7);

	if(temp < 4){
		apb2p = 1;
	}else{

		apb2p = APB1_PreScaler[temp - 4];
	}

	pclk2 = (SystemClk / ahbp)/apb2p;

	return pclk2;
}


uint32_t RCC_GetPLLOutputClock(){

	uint32_t VCOinput = 0;
	uint8_t VCOinputDiv = 0;
	uint16_t VCOoutputMul = 0;
	uint8_t PLLoutputDiv =	0;
	uint32_t PLLoutput = 0;

	//determine PLL input source
	uint8_t check = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_POS;

	if(check){

		VCOinput = 8000000;
	}else{

		VCOinput = 16000000;
	}

	//determine VCO input division factor
	//Division factor must be between 2 and 63
	VCOinputDiv = (RCC->PLLCFGR & RCC_PLLCFGR_M) >>  RCC_PLLCFGR_M_POS;
	if(VCOinputDiv < 2 || VCOinputDiv > 63){

		return -1;
	}

	//determine VCO input multiplication factor
	//it must be between 50 and 432
	VCOoutputMul = (RCC->PLLCFGR & RCC_PLLCFGR_N) >> RCC_PLLCFGR_N_POS;
	if(VCOoutputMul < 50 || VCOoutputMul > 432){

		return -1;
	}

	//determine PLL output division factor
	uint8_t temp = (RCC->PLLCFGR & RCC_PLLCFGR_P) >> RCC_PLLCFGR_P_POS;

	if(!temp){

		PLLoutputDiv = 2;

	}else if(temp == 1){

		PLLoutputDiv = 4;

	}else if(temp == 2){

		PLLoutputDiv = 6;

	}else if(temp == 3){

		PLLoutputDiv = 8;
	}

	PLLoutput = (((double)VCOinput/VCOinputDiv) * VCOoutputMul)/PLLoutputDiv;

	return PLLoutput;
}



void RCC_Set_SYSCLK_PLL_84_MHz(void){

 	PWR_Set_Scale_Mode();

	//SET FLASH LATENCY TO WAIT 7 CYCLES
	FLASH_Set_Latency();

	/*set AHB division factor as 1, APB2 division factor as 2, APB1 division factor as 4 so that AHB clock is 84Mhz, APB1 clock is 21Mhz, APB2  clock is 42 Mhz*/
	RCC->CFGR &= ~(0xF << RCC_CFGR_HPRE_POS);

	RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE1_POS);
	RCC->CFGR |= APB_DIV4 << RCC_CFGR_PPRE1_POS;

	RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE2_POS);
	RCC->CFGR |= APB_DIV2 << RCC_CFGR_PPRE2_POS;

	// set PLL as 84 MHz
	RCC_set_PLL_84_MHz();

	//select PLL as a system clock
	RCC->CFGR &= ~(RCC_CFGR_SW);
	RCC->CFGR |= (RCC_SYSCLK_PLL << RCC_CFGR_SW_POS);

	while(((RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_POS) != RCC_SYSCLK_PLL );

	//turn off HSI
	RCC_HSI_Clock_Control(DISABLE);


}
