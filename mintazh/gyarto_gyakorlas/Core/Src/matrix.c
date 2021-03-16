/*
 * matrix.c
 *
 *  Created on: 2021. febr. 13.
 *      Author: Gergo
 */


#include "main.h"
#include "matrix.h"
#include "lcd.h"

uint8_t row = 1;
uint8_t m_button2 = 12;

unsigned char lcd_num[2];


const unsigned char billtomb[12] = {69, 14, 13, 11, 22, 21, 19, 38, 37, 35, 70, 67};

uint16_t sor_pin[4] = {KB_PC3_OUT_row1_Pin, KB_PC4_OUT_row2_Pin, KB_PC5_OUT_row3_Pin, KB_PC6_OUT_row4_Pin};

uint16_t matrix_pin[7] = {KB_PC6_OUT_row4_Pin, KB_PC5_OUT_row3_Pin, KB_PC4_OUT_row2_Pin, KB_PC3_OUT_row1_Pin, KB_PC2_IN_RIGHT_Pin, KB_PC1_IN_CENTER_Pin, KB_PC0_IN_LEFT_Pin};

GPIO_TypeDef *sor_gpio_port[4] = {KB_PC3_OUT_row1_GPIO_Port, KB_PC4_OUT_row2_GPIO_Port, KB_PC5_OUT_row3_GPIO_Port, KB_PC6_OUT_row4_GPIO_Port};

GPIO_TypeDef *matrix_gpio_port[7] = {KB_PC6_OUT_row4_GPIO_Port, KB_PC5_OUT_row3_GPIO_Port, KB_PC4_OUT_row2_GPIO_Port, KB_PC3_OUT_row1_GPIO_Port, KB_PC2_IN_RIGHT_GPIO_Port, KB_PC1_IN_CENTER_GPIO_Port, KB_PC0_IN_LEFT_GPIO_Port};


void matrix_row(uint8_t row){

	uint8_t i = 0;

	while(i<4){
		if(i==(row-1)) HAL_GPIO_WritePin(sor_gpio_port[i], sor_pin[i], GPIO_PIN_SET);
		else HAL_GPIO_WritePin(sor_gpio_port[i], sor_pin[i], GPIO_PIN_RESET);

		i++;
	}
}


uint8_t matrix_scann(void){

	uint8_t bill = 0;
	uint8_t seged = 0;
	uint8_t i = 0;

	while(i<7){
		seged=HAL_GPIO_ReadPin(matrix_gpio_port[i], matrix_pin[i]);
		bill=(bill<<1)+seged;
		i++;
	}
		return bill;
}


int matrix_search(void){
	uint8_t num=0;
	uint8_t bill=0;

	matrix_row(row);
	HAL_Delay(5);
	bill=matrix_scann();

	while(num<12){

		if(bill==billtomb[num]){
			m_button2=num;


			while(matrix_scann()==billtomb[num]);

			return 0;
		}
		else{

			m_button2=12;
			num++;
		}
	}

	if(row<4) row++;
	else row=1;

	return 1;
}

int get_key(void){
	return m_button2;
}




