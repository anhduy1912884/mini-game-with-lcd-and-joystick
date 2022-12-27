/** 
Edit by modify: Ngoc Hang 
**/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4e // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_init (void)
{
	lcd_send_cmd (0x30);
  HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);
	
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
//	lcd_send_cmd (0x33); /* set 4-bits interface */
//	lcd_send_cmd (0x32);
//	HAL_Delay(50);
//	lcd_send_cmd (0x28); /* start to set LCD function */
//	HAL_Delay(50);
//	lcd_send_cmd (0x01); /* clear display */
//	HAL_Delay(50);
//	lcd_send_cmd (0x06); /* set entry mode */
//	HAL_Delay(50);
//	lcd_send_cmd (0x0c); /* set display to on */	
//	HAL_Delay(50);
//	lcd_send_cmd (0x02); /* move cursor to home and set data address to 0 */
//	HAL_Delay(50);
//	lcd_send_cmd (0x80);
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_clear_display (void)
{
	lcd_send_cmd (0x01); //clear display
}

//void lcd_goto_XY (int row, int col)
//{
//	uint8_t pos_Addr;
//	if(row == 1) 
//	{
//		pos_Addr = 0x80 + row - 1 + col;
//	}
//	else
//	{
//		pos_Addr = 0x80 | (0x40 + col);
//	}
//	lcd_send_cmd(pos_Addr);
//}

void lcd_goto_XY (int row, int col)
{
	uint8_t pos_Addr;
	uint8_t addr ;
	switch (row) {
		case (1) : 
		 {  
			 addr = col - 1 ;
			 break;
		 }
		case (2) : 
		 {  
			 addr = 0x40 + (col -1);
			 break;
		 }
		 
		case (3) : 
		 {   			 
			 addr = 0x14 + (col -1);
			 break;
		 }		 
		case (4) : 
		 {  
			 addr = 0x54 + (col -1);
			 break;
		 }	
	}	
	pos_Addr = 0x80|addr  ;	
	lcd_send_cmd(pos_Addr);
}
