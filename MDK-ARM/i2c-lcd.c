#include "i2c-lcd.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;  // Change to hi2c2 if using I2C2

void lcd_send_cmd (I2C_LCD_HandleTypeDef *lcd, char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, (uint8_t *) data_t, 4, 100);
}

void lcd_send_data (I2C_LCD_HandleTypeDef *lcd, char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit(lcd->hi2c, lcd->address, (uint8_t *) data_t, 4, 100);
}

void lcd_clear (I2C_LCD_HandleTypeDef *lcd)
{
	lcd_send_cmd(lcd, 0x01);
	HAL_Delay(2);
}

void lcd_gotoxy(I2C_LCD_HandleTypeDef *lcd, int x, int y)
{
    int addr = (y == 0) ? 0x80 + x : 0xC0 + x;
    lcd_send_cmd(lcd, addr);
}

void lcd_init (I2C_LCD_HandleTypeDef *lcd)
{
	HAL_Delay(50);
	lcd_send_cmd (lcd, 0x30);
	HAL_Delay(5);
	lcd_send_cmd (lcd, 0x30);
	HAL_Delay(1);
	lcd_send_cmd (lcd, 0x30);
	HAL_Delay(10);
	lcd_send_cmd (lcd, 0x20);  // 4bit mode
	HAL_Delay(10);
	lcd_send_cmd (lcd, 0x28); // Function set --> DL=0 (4 bit mode), N=1 (2 line display) F=0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (lcd, 0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (lcd, 0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (lcd, 0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (lcd, 0x0C); //Display on/off control --> D=1, C=0, B=0  ---> display on
}

void lcd_puts (I2C_LCD_HandleTypeDef *lcd, char *str)
{
	while (*str) lcd_send_data (lcd, *str++);
}