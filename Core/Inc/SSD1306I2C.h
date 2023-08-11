/*
 * SSD1306I2C.h
 *
 *  Created on: 27.05.2023
 *      Author: Lars Henning
 */

#ifndef SRC_SSD1306I2C_H_
#define SRC_SSD1306I2C_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

class SSD1306I2C
{
	public:
		SSD1306I2C();		// Constructor
	protected:
		void SSD1306_I2C_Write(uint8_t address, uint8_t reg, uint8_t data);
		void SSD1306_I2C_WriteMulti(uint8_t address, uint8_t reg, uint8_t* data, uint16_t count);
};



#endif /* SRC_SSD1306I2C_H_ */
