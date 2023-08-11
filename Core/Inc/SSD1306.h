/*
 * SSD1306.h
 *
 *  Created on: 27.05.2023
 *      Author: Lars Henning
 */

#ifndef SRC_SSD1306_H_
#define SRC_SSD1306_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include "fonts.h"
#include "SSD1306I2C.h"


/* SSD1306 settings */
#define SSD1306_I2C_ADDR 0x78		/* I2C address */
#define SSD1306_WIDTH 128			/* SSD1306 width in pixels */
#define SSD1306_HEIGHT 64			/* SSD1306 LCD height in pixels */
#define SSD1306_COLOR_BLACK 0x00	/*!< Black color, no pixel */
#define SSD1306_COLOR_WHITE 0x01  	/*!< Pixel is set. Color depends on LCD */




class SSD1306: public SSD1306I2C
{
	public:
		SSD1306();			// Constructor
	    uint8_t SSD1306_Init(void);
		void SSD1306_GotoXY(uint16_t x, uint16_t y);
		char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color);
		char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color);
		void SSD1306_UpdateScreen(void);
		void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
		void SSD1306_Clear(void);
		void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint8_t color);
		void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t color);
		void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);
	protected:
		uint16_t CurrentX;
		uint16_t CurrentY;
		uint8_t Initialized;
		uint8_t Inverted;
		uint8_t Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
		void SSD1306_Fill(uint8_t color);
};

#endif /* SRC_SSD1306_H_ */
