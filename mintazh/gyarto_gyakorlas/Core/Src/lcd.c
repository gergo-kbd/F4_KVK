/*
 * lcd.c
 *
 *  Created on: 2020. okt. 17.
 *      Author: Gergo
 */

#include "main.h"
#include "lcd.h"

void LCD_clock(void){

	HAL_Delay(6);
	// lcd e 1
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);
	HAL_Delay(5);
	//lcd e 0
	HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}

void LCD_write(uint8_t data){

	//felso 4 bit
	HAL_GPIO_WritePin(LCD_DATA_7_GPIO_Port, LCD_DATA_7_Pin, ((data>>7)&0x01));
	HAL_GPIO_WritePin(LCD_DATA_6_GPIO_Port, LCD_DATA_6_Pin, ((data>>6)&0x01));
	HAL_GPIO_WritePin(LCD_DATA_5_GPIO_Port, LCD_DATA_5_Pin, ((data>>5)&0x01));
	HAL_GPIO_WritePin(LCD_DATA_4_GPIO_Port, LCD_DATA_4_Pin, ((data>>4)&0x01));

	LCD_clock();

	//also 4
	HAL_GPIO_WritePin(LCD_DATA_7_GPIO_Port, LCD_DATA_7_Pin, ((data>>3)&0x01));
	HAL_GPIO_WritePin(LCD_DATA_6_GPIO_Port, LCD_DATA_6_Pin, ((data>>2)&0x01));
	HAL_GPIO_WritePin(LCD_DATA_5_GPIO_Port, LCD_DATA_5_Pin, ((data>>1)&0x01));
	HAL_GPIO_WritePin(LCD_DATA_4_GPIO_Port, LCD_DATA_4_Pin, ((data)&0x01));

	LCD_clock();
}

void LCD_data(uint8_t adat){

	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
	LCD_write(adat);
}

void LCD_cmd(uint8_t cmd){

	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
	LCD_write(cmd);
}

void LCD_init(void){

	LCD_cmd(0x20);
	LCD_clock();
	LCD_clock();
	LCD_clock();
	// mod - 4 bit 2 sor, 5x8pont matrix
	// 3x - nem tudjuk milyen modban volt
	LCD_cmd(0x28);
	LCD_cmd(0x28);
	LCD_cmd(0x28);

	//alaphelyzet
	LCD_cmd(0x02);
	// lcd torles
	LCD_cmd(0x01);
	//kurzor beallitasa
	LCD_cmd(0x08 | (1<<2) | (0<<1) | (0<<0));
}

void LCD_string(char *p){

	while(*p){

		LCD_data(*p++);
	}
}

void LCD_xy(uint8_t x, uint8_t y){

	uint8_t position;

		switch (y)
		{
			case 1:    // 1. sor
			position = LCD_START_DDADR + LCD_ADR_LINE1 + x;
			break;

			case 2:    // 2. sor
			position = LCD_START_DDADR + LCD_ADR_LINE2 + x;
			break;

			case 3:    // 3. sor
			position = LCD_START_DDADR + LCD_ADR_LINE3 + x;
			break;

			case 4:    // 4. sor
			position = LCD_START_DDADR + LCD_ADR_LINE4 + x;
			break;

			default:
			return;
		}
		LCD_cmd(position); // beallitas

}


