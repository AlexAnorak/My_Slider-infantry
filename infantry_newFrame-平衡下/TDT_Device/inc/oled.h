#ifndef _LQOLED_H
#define _LQOLED_H

#include "board.h"
//#include "stdlib.h"

//汉字大小，英文数字大小
#define 	TYPE8X16		1
#define 	TYPE16X16		2
#define 	TYPE6X8			3

//-----------------OLED端口定义----------------  					   

//片选信号
#define OLED_CS(X)   X?GPIO_SetBits(GPIOB, GPIO_Pin_0):GPIO_ResetBits(GPIOB, GPIO_Pin_0)  
//CS管脚请接地
extern void LCD_Put_Column(u8 ,u8 ,u8 data);
void LCD_Line_y(char );
void LCD_WrDat(u8 data);
extern void LCD_Init(u8 ID=0);
extern void LCD_CLS(void);
extern void LCD_CLS_y(char);
extern void LCD_CLS_line_area(u8 start_x,u8 start_y,u8 width);
extern void LCD_P6x8Str(u8,u8,u8 *ch,const u8 *F6x8);
extern void LCD_P8x16Str(u8,u8 ,u8 *ch,const u8 *F8x16);
extern void LCD_P14x16Str(u8 ,u8 ,u8 ch[],const u8 *F14x16_Idx,const u8 *F14x16);
extern void LCD_P16x16Str(u8 ,u8 ,u8 *ch,const u8 *F16x16_Idx,const u8 *F16x16);
//extern void LCD_Print(u8 x, u8 y, u8 *ch);
extern void LCD_PutPixel(u8 ,u8 );
extern void LCD_Print(u8 , u8 , u8 *ch,u8 char_size, u8 ascii_size);
extern void LCD_Rectangle(u8 x1,u8 y1,u8 x2,u8 y2,u8 gif);
extern void Draw_BMP(u8 ,u8 ,const u8 *bmp); 
extern void LCD_Fill(u8 dat);
void OLED_GPIO_Init(void);

#endif

