#ifndef __LCD_INIT_H
#define __LCD_INIT_H



/* Standard Includes */
#include "main.h"
#include "gpio.h"
typedef uint8_t u8;
typedef uint16_t u16;
#define USE_HORIZONTAL 3  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏  两个数字代表从不同的方向显示


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 240
#define LCD_H 320

#else
#define LCD_W 320
#define LCD_H 240
#endif

////电阻屏芯片连接引脚
//#define TCLK 		PAout(0)  	//PA0  SCLK
//#define TDIN 		PAout(1)  	//PA1  MOSI
//#define DOUT 		PAin(5)   	//PA5  MISO
//#define TCS  		PAout(7)  	//PA7  CS2
//#define PEN  		PBin(0)    	//PB0  INT


//-----------------LCD端口定义---------------- 

//#define LCD_SCLK_Clr() GPIOPinWrite(GPIO,GPIO_Pin_0)//SCL=SCLK
//#define LCD_SCLK_Set() GPIO_SetBits(GPIOA,GPIO_Pin_0)

//#define LCD_MOSI_Clr() GPIO_ResetBits(GPIOA,GPIO_Pin_1)//SDA=MOSI
//#define LCD_MOSI_Set() GPIO_SetBits(GPIOA,GPIO_Pin_1)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET) //RES   =  PC9
#define LCD_RES_Set()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)//DC    =  PC7
#define LCD_DC_Set()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)//BLK   =  PC5
#define LCD_BLK_Set()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)

#define LCD_CS_Clr()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)//CS1   =  PC8
#define LCD_CS_Set()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)




void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
void Delay_us(float us);
void Delay_ms(uint32_t ms);
#endif




