#include "lcd.h"
#include "lcd_font.h"
/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(u8 dat) 
{    
    u8 i;
    LCD_CS_Clr();
    for(i=0;i<8;i++)
    {              
        LCD_SCLK_Clr();
        if(dat&0x80)
        {
           LCD_MOSI_Set();
        }
        else
        {
           LCD_MOSI_Clr();
        }
        LCD_SCLK_Set();
        dat<<=1;
    }    
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
    if(USE_HORIZONTAL==0)
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//储存器写
    }
    else if(USE_HORIZONTAL==1)
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//储存器写
    }
    else if(USE_HORIZONTAL==2)
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//储存器写
    }
    else
    {
        LCD_WR_REG(0x2a);//列地址设置
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//行地址设置
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//储存器写
    }
}

void LCD_Init(void)
{
//    LCD_GPIO_Init();//初始化GPIO
    
    LCD_RES_Clr();//复位
    HAL_Delay(100);
    LCD_RES_Set();
    HAL_Delay(100);
    
    LCD_BLK_Set();//打开背光
    HAL_Delay(100);
    
    //************* Start Initial Sequence **********//
    LCD_WR_REG(0x11); //Sleep out 
    HAL_Delay(120);              //Delay 120ms 
    //------------------------------------ST7735S Frame Rate-----------------------------------------// 
    LCD_WR_REG(0xB1); 
    LCD_WR_DATA8(0x05); 
    LCD_WR_DATA8(0x3C); 
    LCD_WR_DATA8(0x3C); 
    LCD_WR_REG(0xB2); 
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x3C); 
    LCD_WR_DATA8(0x3C); 
    LCD_WR_REG(0xB3); 
    LCD_WR_DATA8(0x05); 
    LCD_WR_DATA8(0x3C); 
    LCD_WR_DATA8(0x3C); 
    LCD_WR_DATA8(0x05); 
    LCD_WR_DATA8(0x3C); 
    LCD_WR_DATA8(0x3C); 
    //------------------------------------End ST7735S Frame Rate---------------------------------// 
    LCD_WR_REG(0xB4); //Dot inversion 
    LCD_WR_DATA8(0x03); 
    //------------------------------------ST7735S Power Sequence---------------------------------// 
    LCD_WR_REG(0xC0); 
    LCD_WR_DATA8(0x28); 
    LCD_WR_DATA8(0x08); 
    LCD_WR_DATA8(0x04); 
    LCD_WR_REG(0xC1); 
    LCD_WR_DATA8(0XC0); 
    LCD_WR_REG(0xC2); 
    LCD_WR_DATA8(0x0D); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_REG(0xC3); 
    LCD_WR_DATA8(0x8D); 
    LCD_WR_DATA8(0x2A); 
    LCD_WR_REG(0xC4); 
    LCD_WR_DATA8(0x8D); 
    LCD_WR_DATA8(0xEE); 
    //---------------------------------End ST7735S Power Sequence-------------------------------------// 
    LCD_WR_REG(0xC5); //VCOM 
    LCD_WR_DATA8(0x1A); 
    LCD_WR_REG(0x36); //MX, MY, RGB mode 
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
    else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
    else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
    else LCD_WR_DATA8(0xA0); 
    //------------------------------------ST7735S Gamma Sequence---------------------------------// 
    LCD_WR_REG(0xE0); 
    LCD_WR_DATA8(0x04); 
    LCD_WR_DATA8(0x22); 
    LCD_WR_DATA8(0x07); 
    LCD_WR_DATA8(0x0A); 
    LCD_WR_DATA8(0x2E); 
    LCD_WR_DATA8(0x30); 
    LCD_WR_DATA8(0x25); 
    LCD_WR_DATA8(0x2A); 
    LCD_WR_DATA8(0x28); 
    LCD_WR_DATA8(0x26); 
    LCD_WR_DATA8(0x2E); 
    LCD_WR_DATA8(0x3A); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x01); 
    LCD_WR_DATA8(0x03); 
    LCD_WR_DATA8(0x13); 
    LCD_WR_REG(0xE1); 
    LCD_WR_DATA8(0x04); 
    LCD_WR_DATA8(0x16); 
    LCD_WR_DATA8(0x06); 
    LCD_WR_DATA8(0x0D); 
    LCD_WR_DATA8(0x2D); 
    LCD_WR_DATA8(0x26); 
    LCD_WR_DATA8(0x23); 
    LCD_WR_DATA8(0x27); 
    LCD_WR_DATA8(0x27); 
    LCD_WR_DATA8(0x25); 
    LCD_WR_DATA8(0x2D); 
    LCD_WR_DATA8(0x3B); 
    LCD_WR_DATA8(0x00); 
    LCD_WR_DATA8(0x01); 
    LCD_WR_DATA8(0x04); 
    LCD_WR_DATA8(0x13); 
    //------------------------------------End ST7735S Gamma Sequence-----------------------------// 
    LCD_WR_REG(0x3A); //65k mode 
    LCD_WR_DATA8(0x05); 
    LCD_WR_REG(0x29); //Display on 
} 
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
      函数说明：显示单个字符
      入口数据：x,y显示坐标
                num 要显示的字符
                fc 字的颜色
                bc 字的背景色
                sizey 字号
                mode:  0非叠加模式  1叠加模式
      返回值：  无
******************************************************************************/
void LCD_ShowChar(u16 x,u16 y,u8 num,u16 font_color,u16 back_color,u8 sizey,u8 mode)
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
        if(sizey==12)temp=ascii_1206[num][i];               //调用6x12字体
//        else if(sizey==16)temp=ascii_1608[num][i];         //调用8x16字体
//        else if(sizey==24)temp=ascii_2412[num][i];         //调用12x24字体
//        else if(sizey==32)temp=ascii_3216[num][i];         //调用16x32字体
        else return;
        for(t=0;t<8;t++)
        {
            if(!mode)//非叠加模式
            {
                if(temp&(0x01<<t))LCD_WR_DATA(font_color);
                else LCD_WR_DATA(back_color);
                m++;
                if(m%sizex==0)
                {
                    m=0;
                    break;
                }
            }
            else//叠加模式
            {
                if(temp&(0x01<<t))LCD_DrawPoint(x,y,font_color);//画一个点
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
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 font_color,u16 back_color,u8 sizey,u8 mode)
{         
    while(*p!='\0')
    {       
        LCD_ShowChar(x,y,*p,font_color,back_color,sizey,mode);
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
void LCD_ShowIntNum(u16 x,u16 y,u16 num,u8 len,u16 font_color,u16 back_color,u8 sizey)
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
                LCD_ShowChar(x+t*sizex,y,' ',font_color,back_color,sizey,0);
                continue;
            }else enshow=1; 
              
        }
         LCD_ShowChar(x+t*sizex,y,temp+48,font_color,back_color,sizey,0);
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
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 font_color,u16 back_color,u8 sizey)
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
            LCD_ShowChar(x+(len-2)*sizex,y,'.',font_color,back_color,sizey,0);
            t++;
            len+=1;
        }
         LCD_ShowChar(x+t*sizex,y,temp+48,font_color,back_color,sizey,0);
    }
}



