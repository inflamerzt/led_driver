/*
 * ssd1306.h
 *
 *  Created on: Jan 9, 2023
 *      Author: inflamer
 */

#include "main.h"

static const uint8_t ssd1306_init[] =
{
0x00,   // Control byte (0x00 - command, 0x40 - data)
0xAE,   // Display off
0x20,   // Set Memory Addressing Mode
0x00,//0x10,   // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
0xB0,   // Set Page Start Address for Page Addressing Mode,0-7
0xC8,   // Set COM Output Scan Direction
0x00,   // Set low column address
0x10,   // Set high column address
0x40,   // Set start line address
0x81,   // set contrast control register
0x7F,
0xA1,   // Set segment re-map 0 to 127
0xA6,   // Set normal display
0xA8,   // Set multiplex ratio
59,//0x3F, 	//63 (ROWS(64)-1)
0xA4,   // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
0xD3,   // Set display offset
0x00,   // No offset
0xD5,   // Set display clock divide ratio/oscillator frequency
0xF0,   // Set divide ratio
0xD9,   // Set pre-charge period
0x22,
0xDA,   // Set com pins hardware configuration
0x12,   // Do not use COM left/right remap
0xDB,   // Set vcomh
0x20,   // 0x20,0.77xVcc
//26 - 3
//0x00,   // Control byte (0x00 - command, 0x40 - data)
0x8D,   // Set DC-DC enable

//0x00,   // Control byte (0x00 - command, 0x40 - data)
0x14,   //

//0x00,   // Control byte (0x00 - command, 0x40 - data)
0xAF,   // Turn on SSD1306 panel
//29 - 0
//*************************************************************************** 2x28


0x00,   // Control byte (0x00 - command, 0x40 - data)
0xB0,   // set page 0
0x00,   // set low addr 0
0x10,   // set high addr 0
 };



#define SSD_I2C I2C2

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#define SSD_ADDR 0b01111010
#define SSD_SET_CONTRAST 0x81

#define SSD_ENTIRE_DISPLAY_ON 0xA5
#define SSD_ENTIRE_DISPLAY_OFF 0xA4

#define SSD_SET_NORMAL_DISPLAY 0xA6
#define SSD_SET_INVERSE_DISPLAY 0xA7

#define SSD_SET_DISPLAY_ON 0xAF
#define SSD_SET_DISPLAY_OFF 0xAE

#define SSD_NOP 0xE3

void ssd_init(void);

void ssd_transmit(char data);





#endif /* INC_SSD1306_H_ */
