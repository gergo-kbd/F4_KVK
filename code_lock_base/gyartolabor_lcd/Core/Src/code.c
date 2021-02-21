/*
 * code.c
 *
 *  Created on: 2021. febr. 17.
 *      Author: Gergo
 */

#include "main.h"
#include "lcd.h"
#include "matrix.h"
#include "code.h"

uint8_t pw[] = {5, 2, 3, 4};
uint8_t pw_try[4];
uint8_t pw_block[4];
unsigned char num[1];

uint8_t i=0;
uint8_t state = 1;
unsigned char lcd_num[2];



void pw_read(void)
{
	if(i<4){
		pw_try[i]=get_key();
		i++;
	}
}

// checks if pw match
int compare_pw(void)
{

	uint8_t k;

	for(k=0; k<4; k++){
		if(pw_try[k]!=pw[k]){
			return 1;
		}
	}

	return 0;
}

void bckspc(void)
{
	LCD_cmd(0x01); // clear
	LCD_xy(4,1);

	uint8_t temp=0;

	while(temp<(i-1)){

		sprintf(num, "%d", pw_try[temp]);
		LCD_string(num);
		temp++;
	}
	i--;

}

void read_matrix(void){

	matrix_search();

	int key=get_key();
	if(key<10){
		if(i==4) return; // ha mar negyet beirtunk

		pw_read();
		sprintf(num, "%d", key);
		LCD_string(num);
	}

	if(key==10) {
		if(i!=4) return; // ha nem negyet irtunk be
		if(!compare_pw()){

			LCD_xy(1,2);
			LCD_string("succes");
		}
		else{
			LCD_xy(1,2);
			LCD_string("denied");
		}
		LCD_xy(4+i,1);
	}
	if(key==11) bckspc();
}








