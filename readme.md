/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  *
  *		SSD1306();			// Constructor
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
  *
  *
  *
  *
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */