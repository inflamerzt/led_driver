/*
 * font.h
 *
 *  Created on: 15 февр. 2023 г.
 *      Author: sheiko
 */

#ifndef INC_FONT_H_
#define INC_FONT_H_
#include "main.h"


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



// Font data for Microsoft Sans Serif 12pt
extern const uint8_t microsoftSansSerif_12ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_12ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_12ptDescriptors[];




#endif /* INC_FONT_H_ */
