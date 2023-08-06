#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"


/******************************************************************************
      函数说明：在指定区域填充颜色
      入口数据：xsta,ysta   起始坐标
                xend,yend   终止坐标
								color       要填充的颜色
      返回值：  无
******************************************************************************/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}

/******************************************************************************
      函数说明：在指定位置画点
      入口数据：x,y 画点坐标
                color 点的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y);//设置光标位置 
	LCD_WR_DATA(color);
} 


/******************************************************************************
      函数说明：画线
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   线的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if (delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线 
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);//画点
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/******************************************************************************
      函数说明：画矩形
      入口数据：x1,y1   起始坐标
                x2,y2   终止坐标
                color   矩形的颜色
      返回值：  无
******************************************************************************/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/******************************************************************************
      函数说明：画圆
      入口数据：x0,y0   圆心坐标
                r       半径
                color   圆的颜色
      返回值：  无
******************************************************************************/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);             //3           
		LCD_DrawPoint(x0+b,y0-a,color);             //0           
		LCD_DrawPoint(x0-a,y0+b,color);             //1                
		LCD_DrawPoint(x0-a,y0-b,color);             //2             
		LCD_DrawPoint(x0+b,y0+a,color);             //4               
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6 
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r))//判断要画的点是否过远
		{
			b--;
		}
	}
}

/******************************************************************************
      函数说明：显示汉字串
      入口数据：x,y显示坐标
                *s 要显示的汉字串
                fc 字的颜色
                bc 字的背景色
                sizey 字号 可选 16 24 32
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	while(*s!=0)
	{
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/******************************************************************************
      函数说明：显示单个12x12汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese12x12(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	                         
	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
		           //continue是用于终止本次循环的，也就是本次循环中continue后边的代码都不会再执行，
		           //而是直接跳转到语句的判断部分，进行下一次循环的入口判断。
	}
} 

/******************************************************************************
      函数说明：显示单个16x16汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese16x16(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
  TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 


/******************************************************************************
      函数说明：显示单个24x24汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese24x24(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
} 

/******************************************************************************
      函数说明：显示单个32x32汉字
      入口数据：x,y显示坐标
                *s 要显示的汉字
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChinese32x32(u16 x,u16 y,u8 *s,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++) 
	{
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1)))
		{ 	
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++)
			{
				for(j=0;j<8;j++)
				{	
					if(!mode)//非叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0)
						{
							m=0;
							break;
						}
					}
					else//叠加方式
					{
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
						x++;
						if((x-x0)==sizey)
						{
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}				  	
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/******************************************************************************
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 fc,u16 bc,u8 sizey,u8 mode)
{
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置 
	for(i=0;i<TypefaceNum;i++)
	{ 
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++)
		{
			if(!mode)//非叠加模式
			{
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0)
				{
					m=0;
					break;
				}
			}
			else//叠加模式
			{
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
				x++;
				if((x-x0)==sizex)
				{
					x=x0;
					y++;
					break;
				}
			}
		}
	}   	 	  
}


/******************************************************************************
      函数说明：显示字符串
      入口数据：x,y显示坐标
                *p 要显示的字符串
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 fc,u16 bc,u8 sizey,u8 mode)
{         
	while(*p!='\0')
	{       
		LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
		x+=sizey/2;
		p++;
	}  
}


/******************************************************************************
      函数说明：显示数字
      入口数据：m底数，n指数
      返回值：  无
******************************************************************************/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;
	return result;
}


/******************************************************************************
      函数说明：显示整数变量
      入口数据：x,y显示坐标
                num 要显示整数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
} 


/******************************************************************************
      函数说明：显示两位小数变量
      入口数据：x,y显示坐标
                num 要显示小数变量
                len 要显示的位数
                fc 字的颜色
                bc 字的背景色
                sizey 字号
      返回值：  无
******************************************************************************/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey)
{         	
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/******************************************************************************
      函数说明：显示图片
      入口数据：x,y起点坐标
                length 图片长度
                width  图片宽度
                pic[]  图片数组    
      返回值：  无
******************************************************************************/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[])
{
	u16 i,j;
	u32 k=0;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	for(i=0;i<length;i++)
	{
		for(j=0;j<width;j++)
		{
			LCD_WR_DATA8(pic[k*2]);
			LCD_WR_DATA8(pic[k*2+1]);
			k++;
		}
	}			
}
void Gui_ShowWave(float *InputWaveData, uint16_t c_len, float begin_x, float begin_y, float end_x, float end_y)
{
  /* 坐标系矩形区域域起点(c_x0,c_y0) 终点(c_x1,c_y1) */
  uint16_t c_x0 = (uint16_t)(1.0 * LCD_X_SIZE * begin_x);
  uint16_t c_y0 = (uint16_t)(1.0 * LCD_Y_SIZE * begin_y);
  uint16_t c_x1 = (uint16_t)(1.0 * LCD_X_SIZE * end_x);
  uint16_t c_y1 = (uint16_t)(1.0 * LCD_Y_SIZE * end_y);
  /* 坐标系标度密度*/
  uint16_t ScaleNum = 5;
  // uint16_t perW = (c_x1 - c_x0) / (ScaleNum - 1);
  uint16_t perH = (c_y1 - c_y0) / (ScaleNum - 1);

  /* 颜色 */
  uint16_t BackgroundColor = BLACK;
  uint16_t WaveColor = WHITE;
  uint16_t WordColor = WHITE;

  uint16_t c_AxisColor = BLUE;

  /*数据的最值和刻度*/
  uint32_t max, min;
  float Scale;

  /* 保存上一次的数据，用于清除源波形 */
  static uint16_t StaticWaveData[1000][2];
  static uint16_t StaticWaveLen;

  uint32_t i, j, k;
  static uint8_t Flag;

  uint8_t text[64];

  uint32_t *WaveData=(uint32_t *)malloc(c_len*sizeof(uint32_t));
  for(i=0;i<c_len;i++)
  {
    WaveData[i]=(uint32_t)(InputWaveData[i]*1000);
  }

  if (!Flag)
  {
    LCD_Fill(0,0,320,240,BackgroundColor);

    //   TFT_printf(WordColor, 0, c_x0-16, 0, "V");
    //   TFT_printf(WordColor, 0, h_x0-16, h_y0, "mV");

    //   TFT_printf(WordColor, 0, (uint16_t)(1.0*LCD_X_SIZE*0.05-8), (uint16_t)(1.0*LCD_Y_SIZE*0.1), "THD:");
    //   snprintf(text, 64, "THD:");
    //   Gui_DrawFont_GBK16((uint16_t)(1.0*LCD_X_SIZE*0.05-8), (uint16_t)(1.0*LCD_Y_SIZE*0.1), WordColor, BackgroundColor, text);
  }

  //   TFT_printf(WordColor, 1, (uint16_t)(1.0*LCD_X_SIZE*0.05-8), (uint16_t)(1.0*LCD_Y_SIZE*0.2), "%.3f",THD);
  max = min = WaveData[0];
  for (i = 0; i < c_len; i++)
  {
    if (WaveData[i] > max)
      max = WaveData[i];
    if (WaveData[i] < min)
      min = WaveData[i];
  }
  // max = (uint16_t)max + 2;
  // min = (uint16_t)min;

#if 0
  //新波形清屏输出
  if (Flag)
    for (i = 0; i < StaticWaveLen; i++)
      if (i)
        Gui_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], StaticWaveData[i][0], StaticWaveData[i][1], BackgroundColor);



  for (i = 0; i < c_len; i++)
  {
    uint16_t x = (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0);
    uint16_t y = (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0);
    if (i)
      Gui_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], x, y, WaveColor);
    StaticWaveData[i][0] = x;
    StaticWaveData[i][1] = y;
  }
#elif 0
  // 新波形无清屏输出 V1.0:有缺陷，波形会残缺，新旧波形长度不一致导致清除原波形会覆盖一部分新波形上的点，V2.0:已解决
  static uint16_t x0, y0;
  if (c_len > StaticWaveLen)
  {
    for (i = 0; i < StaticWaveLen; i++)
    {

      if (Flag)
      {
        if (i)
        {
          Gui_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], StaticWaveData[i][0], StaticWaveData[i][1], BackgroundColor);
        }
      }
      if (i)
        Gui_DrawLine(x0, y0, (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0), (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0), WaveColor);
      x0 = (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0);
      y0 = (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0);
    }
    for (i = StaticWaveLen; i < c_len; i++)
    {
      if (i)
        Gui_DrawLine(x0, y0, (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0), (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0), WaveColor);
      x0 = (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0);
      y0 = (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0);
    }
  }
  else
  {
    for (i = 0; i < c_len; i++)
    {
      if (Flag)
      {
        if (i)
        {
          Gui_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], StaticWaveData[i][0], StaticWaveData[i][1], BackgroundColor);
        }
      }
      if (i)
        Gui_DrawLine(x0, y0, (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0), (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0), WaveColor);
      x0 = (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0);
      y0 = (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0);
    }
    for (i = c_len; i < StaticWaveLen; i++)
    {
      if (i)
        Gui_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], StaticWaveData[i][0], StaticWaveData[i][1], BackgroundColor);
    }
  }
  for (i = 0; i < c_len; i++)
  {
    StaticWaveData[i][0] = (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0);
    StaticWaveData[i][1] = (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0);
  }

#else
  // 新波形无清屏输出, V2.0
  static uint16_t x0, y0;
  i = 1;
  j = 0;
  while (1)
  {
    if (i < StaticWaveLen && j < c_len)
    {

      if (StaticWaveData[i - 1][0] > (uint16_t)(1.0 * j / c_len * (c_x1 - c_x0) + c_x0))
      {
        if (j)
          LCD_DrawLine(x0, y0, (uint16_t)(1.0 * j / c_len * (c_x1 - c_x0) + c_x0), (uint16_t)(1.0 * (max - WaveData[j]) / (max - min) * (c_y1 - c_y0) + c_y0), WaveColor);
        x0 = (uint16_t)(1.0 * j / c_len * (c_x1 - c_x0) + c_x0);
        y0 = (uint16_t)(1.0 * (max - WaveData[j]) / (max - min) * (c_y1 - c_y0) + c_y0);
        j++;
      }
      else
      {
        if (Flag)
        {
          if (i)
          {
            LCD_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], StaticWaveData[i][0], StaticWaveData[i][1], BackgroundColor);
          }
        }
        i++;
      }
    }
    else if (i < StaticWaveLen)
    {
      if (Flag)
      {
        if (i)
        {
          LCD_DrawLine(StaticWaveData[i - 1][0], StaticWaveData[i - 1][1], StaticWaveData[i][0], StaticWaveData[i][1], BackgroundColor);
        }
      }
      i++;
    }
    else if (j < c_len)
    {
      if (j)
        LCD_DrawLine(x0, y0, (uint16_t)(1.0 * j / c_len * (c_x1 - c_x0) + c_x0), (uint16_t)(1.0 * (max - WaveData[j]) / (max - min) * (c_y1 - c_y0) + c_y0), WaveColor);
      x0 = (uint16_t)(1.0 * j / c_len * (c_x1 - c_x0) + c_x0);
      y0 = (uint16_t)(1.0 * (max - WaveData[j]) / (max - min) * (c_y1 - c_y0) + c_y0);
      j++;
    }
    else
    {
      break;
    }
  }
  for (i = 0; i < c_len; i++)
  {
    StaticWaveData[i][0] = (uint16_t)(1.0 * i / c_len * (c_x1 - c_x0) + c_x0);
    StaticWaveData[i][1] = (uint16_t)(1.0 * (max - WaveData[i]) / (max - min) * (c_y1 - c_y0) + c_y0);
  }

#endif

  StaticWaveLen = c_len;
  Scale = 1.0 * (max - min) / ScaleNum;
  for (i = 0; i < ScaleNum; i++)
  {
    //  TFT_printf(WordColor, 1, c_x0-36, c_y0 - 8 + i * perH, "%.1f",(float)((max - Scale * i)/4096*3.3));
    snprintf(text, 5, "%.2f", (float)((max - Scale * i) / 1000));
    LCD_ShowChinese16x16(c_x0 - 12 * 4, c_y0 - 8 + i * perH,text, WordColor, BackgroundColor,16,0);
  }

  LCD_DrawLine((c_x0), c_y1, (c_x0), c_y0, c_AxisColor);
  LCD_DrawLine((c_x0), (c_y1 + c_y0) / 2, (c_x1), (c_y1 + c_y0) / 2, c_AxisColor);
  LCD_DrawLine((c_x0), c_y0, (c_x0 - 3), c_y0 + 3, c_AxisColor);
  LCD_DrawLine((c_x0), c_y0, (c_x0 + 3), c_y0 + 3, c_AxisColor);
  LCD_DrawLine((c_x1), (c_y1 + c_y0) / 2, (c_x1 - 3), (c_y1 + c_y0) / 2 - 3, c_AxisColor);
  LCD_DrawLine((c_x1), (c_y1 + c_y0) / 2, (c_x1 - 3), (c_y1 + c_y0) / 2 + 3, c_AxisColor);

  if (!Flag)
  {
    Flag++;
  }

  free(WaveData);

}



