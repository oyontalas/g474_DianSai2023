#include "lcd_init.h"
#include "spi.h"
#include "math.h"
#include "stm32g4xx.h"
/* Standard Includes */

#include <stdio.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


void LCD_Writ_Bus(u8 dat) 
{	
	LCD_CS_Clr();
	HAL_SPI_Transmit(&hspi3, &dat, 1, 0xff);
  LCD_CS_Set();	
}


/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}

/******************************************************************************
      函数说明：LCD写入数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
}

void LCD_Init(void)
{
	//LCD_GPIO_Init();//初始化GPIO
	
	LCD_RES_Clr();//复位
	HAL_Delay(20);
	LCD_RES_Set();
	HAL_Delay(20);
	
	LCD_BLK_Set();//打开背光
  HAL_Delay(100);
	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	HAL_Delay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0xCF);  
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0xD9); 
	LCD_WR_DATA8(0X30); 
	 
	LCD_WR_REG(0xED);  
	LCD_WR_DATA8(0x64); 
	LCD_WR_DATA8(0x03); 
	LCD_WR_DATA8(0X12); 
	LCD_WR_DATA8(0X81); 
	 
	LCD_WR_REG(0xE8);  
	LCD_WR_DATA8(0x85); 
	LCD_WR_DATA8(0x10); 
	LCD_WR_DATA8(0x78); 
	 
	LCD_WR_REG(0xCB);  
	LCD_WR_DATA8(0x39); 
	LCD_WR_DATA8(0x2C); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x34); 
	LCD_WR_DATA8(0x02); 
	 
	LCD_WR_REG(0xF7);  
	LCD_WR_DATA8(0x20); 
	 
	LCD_WR_REG(0xEA);  
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x00); 
	 
	LCD_WR_REG(0xC0);    //Power control 
	LCD_WR_DATA8(0x21);   //VRH[5:0] 
	 
	LCD_WR_REG(0xC1);    //Power control 
	LCD_WR_DATA8(0x12);   //SAP[2:0];BT[3:0] 
	 
	LCD_WR_REG(0xC5);    //VCM control 
	LCD_WR_DATA8(0x32); 
	LCD_WR_DATA8(0x3C); 
	 
	LCD_WR_REG(0xC7);    //VCM control2 
	LCD_WR_DATA8(0XC1); 
	 
	LCD_WR_REG(0x36);    // Memory Access Control 
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x78);
	else LCD_WR_DATA8(0xA8);
	 
	LCD_WR_REG(0x3A);   
	LCD_WR_DATA8(0x55); 

	LCD_WR_REG(0xB1);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x18); 
	 
	LCD_WR_REG(0xB6);    // Display Function Control 
	LCD_WR_DATA8(0x0A); 
	LCD_WR_DATA8(0xA2); 

	 
	 
	LCD_WR_REG(0xF2);    // 3Gamma Function Disable 
	LCD_WR_DATA8(0x00); 
	 
	LCD_WR_REG(0x26);    //Gamma curve selected 
	LCD_WR_DATA8(0x01); 
	 
	LCD_WR_REG(0xE0);    //Set Gamma 
	LCD_WR_DATA8(0x0F); 
	LCD_WR_DATA8(0x20); 
	LCD_WR_DATA8(0x1E); 
	LCD_WR_DATA8(0x09); 
	LCD_WR_DATA8(0x12); 
	LCD_WR_DATA8(0x0B); 
	LCD_WR_DATA8(0x50); 
	LCD_WR_DATA8(0XBA); 
	LCD_WR_DATA8(0x44); 
	LCD_WR_DATA8(0x09); 
	LCD_WR_DATA8(0x14); 
	LCD_WR_DATA8(0x05); 
	LCD_WR_DATA8(0x23); 
	LCD_WR_DATA8(0x21); 
	LCD_WR_DATA8(0x00); 
	 
	LCD_WR_REG(0XE1);    //Set Gamma 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x19); 
	LCD_WR_DATA8(0x19); 
	LCD_WR_DATA8(0x00); 
	LCD_WR_DATA8(0x12); 
	LCD_WR_DATA8(0x07); 
	LCD_WR_DATA8(0x2D); 
	LCD_WR_DATA8(0x28); 
	LCD_WR_DATA8(0x3F); 
	LCD_WR_DATA8(0x02); 
	LCD_WR_DATA8(0x0A); 
	LCD_WR_DATA8(0x08); 
	LCD_WR_DATA8(0x25); 
	LCD_WR_DATA8(0x2D); 
	LCD_WR_DATA8(0x0F); 
	LCD_WR_REG(0x29);    //Display on 
}









