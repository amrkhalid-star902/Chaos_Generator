/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Jan 28, 2023
 *      Author: AKhal
 */

#include "stm32f407xx_usart_driver.h"

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){

	//variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part , F_part;

	uint32_t tempreg = 0;

	if(pUSARTx == USART1 || pUSARTx == USART6){

		PCLKx = RCC_GetPCLK2Value();
	}else{

		PCLKx = RCC_GetPCLK1Value();
	}

	//check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){

		 //OVER8 = 1 , over sampling by 8
		 //usartdiv is multiplied by 100 to
		 //make calculations easier
		 usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	}else{

		 //OVER8 = 0 , over sampling by 16
		 //usartdiv is multiplied by 100 to
		 //make calculations easier
		 usartdiv = ((25 * PCLKx) / (4 * BaudRate));

	}

	//calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position . refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){

		//OVER8 = 1 , over sampling by 8
		F_part = (((F_part * 8) + 50)/100) & ((uint8_t)0x07);

	}else{

		//OVER8 = 0 , over sampling by 16
		F_part = (((F_part * 16) + 50)/100) & ((uint8_t)0x0F);
	}

	 //Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	pUSARTx->BBR = tempreg;

}


void USART_PeriClockControl(USART_RegDef_t *pUSARTx , uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pUSARTx == USART1){

			USART1_PCLK_EN();
		}else if(pUSARTx == USART2){

			USART2_PCLK_EN();

		}else if(pUSARTx == USART3){

			USART3_PCLK_EN();

		}else if(pUSARTx == UART4){

			UART4_PCLK_EN();

		}else if(pUSARTx == UART5){

			UART5_PCLK_EN();

		}else if(pUSARTx == USART6){

			USART6_PCLK_EN();
		}
	}else{

		if(pUSARTx == USART1){

			USART1_PCLK_DI();
		}else if(pUSARTx == USART2){

			USART2_PCLK_DI();

		}else if(pUSARTx == USART3){

			USART3_PCLK_DI();

		}else if(pUSARTx == UART4){

			UART4_PCLK_DI();

		}else if(pUSARTx == UART5){

			UART5_PCLK_DI();

		}else if(pUSARTx == USART6){

			USART6_PCLK_DI();
		}

	}
}


void USART_Init(USART_Handle_t *pUSARTHandle){

	uint32_t temp = 0;

	//enable the clock for USARTx peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){

		//enable receiver bit field
		temp |= (1 << USART_CR1_RE);

	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){

		//enable transmitter bit field
		temp |= (1 << USART_CR1_TE);

	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){

		temp |= (1 << USART_CR1_TE) | (1 << USART_CR1_RE);
	}

	//configure the world length configuration bit
	temp |= (pUSARTHandle->USART_Config.USART_WordLength) << USART_CR1_M;

	//configure the parity bit field
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){

		//enable parity bit
		temp |= (1 << USART_CR1_PCE);

		//the parity bit by default even
	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		//enable parity bit
		temp |= (1 << USART_CR1_PCE);

		temp |= (1 << USART_CR1_PS);

	}

	pUSARTHandle->pUSARTx->CR1 = temp;

	/******************************** Configuration of CR2******************************************/

	temp = 0;

	//configure number of stop bits
	temp |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	pUSARTHandle->pUSARTx->CR2 = temp;

	/******************************** Configuration of CR3******************************************/

	temp = 0;
	//configuration of hardware control flow
	if(pUSARTHandle->USART_Config.USART_HWFLowControl == USART_HW_FLOW_CTRL_CTS){

		//Implement the code to enable CTS flow control
		temp |= (1 << USART_CR3_CTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFLowControl == USART_HW_FLOW_CTRL_RTS){

		//Implement the code to enable RTS flow control
		temp |= (1 << USART_CR3_RTSE);

	}else if(pUSARTHandle->USART_Config.USART_HWFLowControl == USART_HW_FLOW_CTRL_CTS_RTS){
			//Enable both CTS and RTS
			temp |= (1 << USART_CR3_RTSE) | (1 << USART_CR3_CTSE);


	}

	pUSARTHandle->pUSARTx->CR3 = temp;

	/******************************** Configuration of BRR(Baudrate register)******************************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}


void USART_DeInit(USART_RegDef_t *pUSARTX){

		if(pUSARTX == USART1){

			USART1_REG_RESET();

		}if(pUSARTX == USART2){

			USART2_REG_RESET();

		}if(pUSARTX == USART3){

			USART3_REG_RESET();

		}if(pUSARTX == UART4){

			UART4_REG_RESET();

		}if(pUSARTX == UART5){

			UART5_REG_RESET();

		}if(pUSARTX == USART6){

			USART6_REG_RESET();
		}
}



void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		pUSARTx->CR1 |= (1 << USART_CR1_UE);

	}else{

		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);

	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){

	if(pUSARTx->SR & StatusFlagName){

		return SET;
	}

	return RESET;
}


void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint16_t *pdata;

	for(int i = 0 ; i < Len ; i++){

		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//Check parity bit
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE){

				//this means that all 9 bits are data from the user
				pTxBuffer+=2;

			}else{

				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;

			}
		}else{

			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (0xFF));

			pTxBuffer++;
		}

	}

	//wait for TC to be set
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}






void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len){

	for(uint32_t i = 0 ; i < Len ; i++){

		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));
		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

			//check if parity is used or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE){

				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & ((uint16_t)0x01FF));
				pRxBuffer+=2;

			}else{

				//Parity is used, so 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}else{

			//8 bit data frame are going to be received

			//check if parity is enabled
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE){

				//all 8 bits are going to be read
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);

			}else{

				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);


			}
			pRxBuffer++;

		}

	}
}



uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len){

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX){

		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}




uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len){

	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX){

		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);


	}

	return rxstate;

}


void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){

	pUSARTx->SR &= ~(StatusFlagName);
}


void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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



void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	//1. first lets find out the ipriority(ipr) register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQNumber << shift_amount);
}



void USART_IRQHandling(USART_Handle_t *pUSARTHandle){

	uint32_t temp1,temp2,temp3;

	uint16_t *pdata;

	/*************************Check for TC flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2){

		//This interrupt because of TC
		//close transmission and call application callback if TxLen is zero
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){

			//if Txlen zero then the transmission will be closed
			if(!pUSARTHandle->TxLen){

				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE );

				pUSARTHandle->TxBusyState = USART_READY;

				pUSARTHandle->pTxBuffer = NULL;

				pUSARTHandle->TxLen = 0;

				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);

			}
		}
	}

	/*************************Check for TXE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2){

		//This interrupt because of TXE
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX){

			if(pUSARTHandle->TxLen > 0){

				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for parity bit
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE){

						//No parity is used in this transfer , so 9bits of user data will be sent
						pUSARTHandle->pTxBuffer+=2;
						pUSARTHandle->TxLen-=2;
					}else{

						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;

					}


				}else{

					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}
			if(pUSARTHandle->TxLen == 0){

				//disable TXE interrupt
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){

		//this interrupt due to rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX){

			if(pUSARTHandle->RxLen > 0){

				//check data frame length
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){

					//check for parity bit
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE){

						//No parity is used , so all 9bits will be of user data
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR) & ((uint16_t)0x01FF);

						//increment buffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}else{

						//parity is used so 8bits only will be of the user data
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR)  & ((uint8_t)0xFF);
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=1;
					}
				}else{

					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_DISABLE){

						//all 8 bits are user data
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR)  & ((uint8_t)0xFF);

					}else{

						//Parity is used , so 7 bits only are user data
						*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR)  & ((uint8_t)0x7F);

					}

					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen-=1;
				}
			}

			if(!pUSARTHandle->RxLen){

				//disable interrupt due to rxne
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

	/*************************Check for CTS flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3){

		//clear cts flag
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_CTS);

		//notify callback function
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}


	/*************************Check for IDLE detection flag ********************************************/
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2){

		//clear IDLE flag
		temp1 = pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);

		//notify application call back function
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){

		//the flag will be cleared using callback function
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	/*************************Check for Error Flag ********************************************/
	//this part of code is excured during multi buffer communication

	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE);

	if(temp2){

		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE)){

			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}else if(temp1 & (1 << USART_SR_NE)){

			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}else if(temp1 & (1 << USART_SR_ORE)){

			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}


__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event)
{

}
