#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#define  u8  uint8_t
#define  u16 uint16_t
#define  u32 uint32_t

#define USE_HORIZONTAL 1  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
                          //0,1,2,3分别控制4个不同的方向显示


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 128
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 128
#endif


//-----------------LCD端口定义---------------- 

#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCLK_GPIO_Port,LCD_SCLK_Pin,GPIO_PIN_RESET)//SCLK
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCLK_GPIO_Port,LCD_SCLK_Pin,GPIO_PIN_SET)

#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_MOSI_GPIO_Port,LCD_MOSI_Pin,GPIO_PIN_RESET)//MOSI
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_MOSI_GPIO_Port,LCD_MOSI_Pin,GPIO_PIN_SET)

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin,GPIO_PIN_RESET)//RES
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port,LCD_RES_Pin,GPIO_PIN_SET)

#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_RESET)//DC
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port,LCD_DC_Pin,GPIO_PIN_SET)
              
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_RESET)//CS
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port,LCD_CS_Pin,GPIO_PIN_SET)

#define LCD_BLK_Clr()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin,GPIO_PIN_RESET)//BLK
#define LCD_BLK_Set()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,LCD_BLK_Pin,GPIO_PIN_SET)

//画笔颜色
#define WHITE              0xFFFF
#define BLACK              0x0000      
#define BLUE            0x001F  
#define BRED            0XF81F
#define GRED                   0XFFE0
#define GBLUE                  0X07FF
#define RED                0xF800
#define MAGENTA            0xF81F
#define GREEN              0x07E0
#define CYAN               0x7FFF
#define YELLOW             0xFFE0
#define BROWN                  0XBC40 //棕色
#define BRRED                  0XFC07 //棕红色
#define GRAY                   0X8430 //灰色
#define DARKBLUE           0X01CF    //深蓝色
#define LIGHTBLUE       0X7D7C    //浅蓝色  
#define GRAYBLUE        0X5458 //灰蓝色
#define LIGHTGREEN      0X841F //浅绿色
#define LGRAY                  0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE       0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          0X2B12 //浅棕蓝色(选择条目的反色)


void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化

void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);//指定区域填充颜色
void LCD_DrawPoint(u16 x,u16 y,u16 color);//在指定位置画一个点
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);//在指定位置画一条线
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);//在指定位置画一个矩形
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);//在指定位置画一个圆


void LCD_ShowChar(u16 x,u16 y,u8 num,
                    u16 font_color,u16 back_color,u8 sizey,u8 mode);//显示一个字符
void LCD_ShowString(u16 x,u16 y,const u8 *p,
                    u16 font_color,u16 back_color,u8 sizey,u8 mode);//显示字符串
u32 mypow(u8 m,u8 n);//求幂
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,
                    u16 font_color,u16 back_color,u8 sizey);//显示整数变量
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,
                    u16 font_color,u16 back_color,u8 sizey);//显示两位小数变量



#endif
