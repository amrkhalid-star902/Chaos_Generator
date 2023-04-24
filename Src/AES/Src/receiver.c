#include <stdio.h>
#include <string.h>
#include <time.h>
#include "aes.h"
#include "stm32f407xx.h"

uint8_t keys[16][1000];

static void printhex(uint8_t * str){

    uint8_t len = 16;

    unsigned char i;

    for(i = 0 ; i < len ; i++){

        printf("%.2x" , str[i]);
    }

    printf("\n");
}

/*
 *  PD5 -> Tx
 *  pD6 -> Rx
 */

USART_Handle_t usart_handle;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void USART2_Init(void){

	usart_handle.pUSARTx = USART2;
	usart_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart_handle.USART_Config.USART_HWFLowControl = USART_HW_FLOW_CTRL_NONE;
	usart_handle.USART_Config.USART_Mode = USART_MODE_ONLY_RX;
	usart_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart_handle.USART_Config.USART_ParityControl = USART_PARITY_EN_DISABLE;
	usart_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&usart_handle);

}


void USART2_GPIOInit(void){

	GPIO_Handle_t usart_gpio;
	usart_gpio.pGPIOx = GPIOD;
	usart_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usart_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpio.GPIO_PinConfig.PinMode = GPIO_MODE_ALTFN;
	usart_gpio.GPIO_PinConfig.PinSpeed = GPIO_SPEED_FAST;

	//USART2 Tx
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&usart_gpio);

	//USART2 Rx
	usart_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&usart_gpio);

}

/*void WriteFile(FILE *file , uint8_t * key , uint32_t idx){

	fprintf(file , "Key number : %lu\n" , idx);

	for(int i = 0 ; i < 16 ; i++)
		fprintf(file , "key:%u" , key[i] );

	fprintf(file , "\n");



}*/




int main(){

	USART2_GPIOInit();

	USART2_Init();

	USART_PeripheralControl(USART2, ENABLE);

	uint8_t key[16];

	uint32_t i = 0;

	/*char * filename = "keys.txt";

    // open the file for writing
    FILE *fp = fopen(filename, "w");

    if (fp == NULL)
    {
        printf("Error opening the file %s", filename);
        return -1;
    }*/

	while(i < 1000000){

		USART_ReceiveData(&usart_handle,key, 16);

	    //WriteFile(fp, key, i);
		printf("key:%lu\n" , i);
	    printhex(key);
	    printf("\n");

	    i++;
	}



	return 0;
}
