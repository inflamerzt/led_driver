/*
 * ssd1306.c
 *
 *  Created on: Jan 9, 2023
 *      Author: inflamer
 */

#include "ssd1306.h"

void ssd_init(void){

	//init sequence (from datasheet)
	//Set MUX Ratio A8h, 3Fh

	//Set Display Offset D3h, 00h

	//Set Display Start Line 40h

	//Set Segment re-map A0h/A1h

	//Set COM Output Scan Direction C0h/C8h

	//Set COM Pins hardware configuration DAh, 02

	//Set Contrast Control 81h, 7Fh

	//Disable Entire Display On A4h

	//Set Normal Display A6h

	//Set Osc Frequency D5h, 80h

	//Enable charge pump regulator 8Dh, 14h

	//Display On AFh

};


void ssd_transmit(char data){

	//LL_I2C_SetSlaveAddr(SSD_I2C, SSD_ADDR);
	//while(!LL_I2C_IsActiveFlag_TXE(SSD_I2C)){};
	//LL_I2C_TransmitData8(SSD_I2C, data);
	//while(!LL_I2C_IsActiveFlag_TXE(SSD_I2C)){};


};
