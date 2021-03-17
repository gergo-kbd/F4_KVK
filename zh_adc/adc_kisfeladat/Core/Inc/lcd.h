/*
 * lcd.h
 *
 *  Created on: 2020. okt. 17.
 *      Author: Gergo
 */

#ifndef LCD_H_
#define LCD_H_

#define LCD_ADR_LINE1         0x00
#define LCD_ADR_LINE2         0x40
#define LCD_ADR_LINE3         0x10
#define LCD_ADR_LINE4         0x50
#define LCD_START_DDADR			0x80

void LCD_clock(void);
void LCD_write(uint8_t data);
void LCD_data(uint8_t adat);
void LCD_cmd(uint8_t cmd);
void LCD_init(void);
void LCD_string(char *p);
void LCD_xy(uint8_t x, uint8_t y);


#endif /* LCD_H_ */
