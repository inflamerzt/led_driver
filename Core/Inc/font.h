/*
 * font.h
 *
 *  Created on: 15 февр. 2023 г.
 *      Author: sheiko
 */

#ifndef INC_FONT_H_
#define INC_FONT_H_
#include "main.h"
#define uint_8 uint8_t

// Character's info struct.
typedef struct FontTable {
  uint16_t    width; // Character's width.
  uint16_t    start; // Index of first character's byte.
} FONT_CHAR_INFO;

// Font's info struct.
typedef struct
{
  uint8_t Height;                  // Character's height.
  uint8_t FirstChar;               // First character.
  uint8_t LastChar;                // Last character.
  uint8_t FontSpace;               // Width of space character.
  const FONT_CHAR_INFO *FontTable; // Character descriptor array
  const uint8_t *FontBitmaps;      // Character bitmap array
} FONT_INFO;



// Font data for Lucida Console 18pt
extern const uint_8 lucidaConsole_18ptBitmaps[];
extern const FONT_INFO lucidaConsole_18ptFontInfo;
extern const FONT_CHAR_INFO lucidaConsole_18ptDescriptors[];







#endif /* INC_FONT_H_ */
