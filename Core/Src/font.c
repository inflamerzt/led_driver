#include "font.h"

//
//  Font data for Microsoft Sans Serif 12pt
//

// Character bitmaps for Microsoft Sans Serif 12pt
const uint8_t microsoftSansSerif_12ptBitmaps[] =
{
	// @0 '0' (7 pixels wide)
	//   ###
	//  #   #
	//  #   #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   #
	//  #   #
	//   ###
	//
	//
	//
	//
	0xF8, 0x06, 0x01, 0x01, 0x01, 0x06, 0xF8,
	0x01, 0x06, 0x08, 0x08, 0x08, 0x06, 0x01,

	// @14 '1' (3 pixels wide)
	//   #
	// ###
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//
	//
	//
	//
	0x02, 0x02, 0xFF,
	0x00, 0x00, 0x0F,

	// @20 '2' (7 pixels wide)
	//   ###
	//  #   #
	// #     #
	//       #
	//       #
	//      #
	//     #
	//    #
	//   #
	//  #
	// #
	// #######
	//
	//
	//
	//
	0x04, 0x02, 0x01, 0x81, 0x41, 0x22, 0x1C,
	0x0C, 0x0A, 0x09, 0x08, 0x08, 0x08, 0x08,

	// @34 '3' (7 pixels wide)
	//   ###
	//  #   #
	// #     #
	//       #
	//      #
	//    ##
	//      #
	//       #
	//       #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0x04, 0x02, 0x01, 0x21, 0x21, 0x52, 0x8C,
	0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @48 '4' (7 pixels wide)
	//      #
	//     ##
	//     ##
	//    # #
	//   #  #
	//   #  #
	//  #   #
	// #    #
	// #######
	//      #
	//      #
	//      #
	//
	//
	//
	//
	0x80, 0x40, 0x30, 0x08, 0x06, 0xFF, 0x00,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x0F, 0x01,

	// @62 '5' (7 pixels wide)
	// #######
	// #
	// #
	// #
	// # ###
	// ##   #
	//       #
	//       #
	//       #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0x3F, 0x21, 0x11, 0x11, 0x11, 0x21, 0xC1,
	0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @76 '6' (7 pixels wide)
	//   ###
	//  #   #
	// #     #
	// #
	// # ###
	// ##   #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0xFC, 0x22, 0x11, 0x11, 0x11, 0x22, 0xC4,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @90 '7' (7 pixels wide)
	// #######
	//       #
	//       #
	//      #
	//      #
	//      #
	//     #
	//     #
	//     #
	//    #
	//    #
	//    #
	//
	//
	//
	//
	0x01, 0x01, 0x01, 0x01, 0xC1, 0x39, 0x07,
	0x00, 0x00, 0x00, 0x0E, 0x01, 0x00, 0x00,

	// @104 '8' (7 pixels wide)
	//   ###
	//  #   #
	// #     #
	// #     #
	//  #   #
	//   ###
	//  #   #
	// #     #
	// #     #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0x8C, 0x52, 0x21, 0x21, 0x21, 0x52, 0x8C,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @118 '9' (7 pixels wide)
	//   ###
	//  #   #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//       #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0x3C, 0x42, 0x81, 0x81, 0x81, 0x42, 0xFC,
	0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @132 'A' (11 pixels wide)
	//      #
	//      #
	//     # #
	//     # #
	//    #   #
	//    #   #
	//   #     #
	//   #######
	//  #       #
	//  #       #
	// #         #
	// #         #
	//
	//
	//
	//
	0x00, 0x00, 0xC0, 0xB0, 0x8C, 0x83, 0x8C, 0xB0, 0xC0, 0x00, 0x00,
	0x0C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0C,

	// @154 'B' (8 pixels wide)
	// ######
	// #     #
	// #      #
	// #      #
	// #     #
	// ######
	// #     #
	// #      #
	// #      #
	// #      #
	// #     #
	// ######
	//
	//
	//
	//
	0xFF, 0x21, 0x21, 0x21, 0x21, 0x21, 0x52, 0x8C,
	0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @170 'C' (10 pixels wide)
	//    ####
	//  ##    ##
	//  #      #
	// #        #
	// #
	// #
	// #
	// #
	// #        #
	//  #      #
	//  ##    ##
	//    ####
	//
	//
	//
	//
	0xF8, 0x06, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x06, 0x08,
	0x01, 0x06, 0x04, 0x08, 0x08, 0x08, 0x08, 0x04, 0x06, 0x01,

	// @190 'D' (9 pixels wide)
	// ######
	// #     ##
	// #      #
	// #       #
	// #       #
	// #       #
	// #       #
	// #       #
	// #       #
	// #      #
	// #     ##
	// ######
	//
	//
	//
	//
	0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x06, 0xF8,
	0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x04, 0x06, 0x01,

	// @208 'E' (8 pixels wide)
	// ########
	// #
	// #
	// #
	// #
	// #######
	// #
	// #
	// #
	// #
	// #
	// ########
	//
	//
	//
	//
	0xFF, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x01,
	0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

	// @224 'F' (8 pixels wide)
	// ########
	// #
	// #
	// #
	// #
	// #######
	// #
	// #
	// #
	// #
	// #
	// #
	//
	//
	//
	//
	0xFF, 0x21, 0x21, 0x21, 0x21, 0x21, 0x21, 0x01,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @240 'G' (10 pixels wide)
	//    ####
	//  ##    ##
	//  #      #
	// #        #
	// #
	// #
	// #     ####
	// #        #
	// #        #
	//  #      ##
	//  ##    ###
	//    ####  #
	//
	//
	//
	//
	0xF8, 0x06, 0x02, 0x01, 0x01, 0x01, 0x41, 0x42, 0x46, 0xC8,
	0x01, 0x06, 0x04, 0x08, 0x08, 0x08, 0x08, 0x04, 0x06, 0x0F,

	// @260 'H' (8 pixels wide)
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// ########
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	//
	//
	//
	//
	0xFF, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0xFF,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,

	// @276 'I' (1 pixels wide)
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	//
	//
	//
	//
	0xFF,
	0x0F,

	// @278 'J' (6 pixels wide)
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	// #    #
	// #    #
	// #    #
	//  ####
	//
	//
	//
	//
	0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0x07, 0x08, 0x08, 0x08, 0x08, 0x07,

	// @290 'K' (8 pixels wide)
	// #     #
	// #    #
	// #   #
	// #  #
	// # #
	// ##
	// # #
	// #  #
	// #   #
	// #    #
	// #     #
	// #      #
	//
	//
	//
	//
	0xFF, 0x20, 0x50, 0x88, 0x04, 0x02, 0x01, 0x00,
	0x0F, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08,

	// @306 'L' (7 pixels wide)
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #######
	//
	//
	//
	//
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x0F, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,

	// @320 'M' (9 pixels wide)
	// #       #
	// #       #
	// ##     ##
	// ##     ##
	// # #   # #
	// # #   # #
	// #  # #  #
	// #  # #  #
	// #   #   #
	// #   #   #
	// #       #
	// #       #
	//
	//
	//
	//
	0xFF, 0x0C, 0x30, 0xC0, 0x00, 0xC0, 0x30, 0x0C, 0xFF,
	0x0F, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x0F,

	// @338 'N' (7 pixels wide)
	// #     #
	// ##    #
	// ##    #
	// # #   #
	// # #   #
	// #  #  #
	// #  #  #
	// #   # #
	// #   # #
	// #    ##
	// #    ##
	// #     #
	//
	//
	//
	//
	0xFF, 0x06, 0x18, 0x60, 0x80, 0x00, 0xFF,
	0x0F, 0x00, 0x00, 0x00, 0x01, 0x06, 0x0F,

	// @352 'O' (10 pixels wide)
	//    ####
	//  ##    ##
	//  #      #
	// #        #
	// #        #
	// #        #
	// #        #
	// #        #
	// #        #
	//  #      #
	//  ##    ##
	//    ####
	//
	//
	//
	//
	0xF8, 0x06, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x06, 0xF8,
	0x01, 0x06, 0x04, 0x08, 0x08, 0x08, 0x08, 0x04, 0x06, 0x01,

	// @372 'P' (8 pixels wide)
	// ######
	// #     #
	// #      #
	// #      #
	// #      #
	// #     #
	// ######
	// #
	// #
	// #
	// #
	// #
	//
	//
	//
	//
	0xFF, 0x41, 0x41, 0x41, 0x41, 0x41, 0x22, 0x1C,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @388 'Q' (10 pixels wide)
	//    ####
	//  ##    ##
	//  #      #
	// #        #
	// #        #
	// #        #
	// #        #
	// #        #
	// #    #   #
	//  #    # #
	//  ##    ##
	//    #### #
	//          #
	//
	//
	//
	0xF8, 0x06, 0x02, 0x01, 0x01, 0x01, 0x01, 0x02, 0x06, 0xF8,
	0x01, 0x06, 0x04, 0x08, 0x08, 0x09, 0x0A, 0x04, 0x0E, 0x11,

	// @408 'R' (9 pixels wide)
	// ######
	// #     #
	// #      #
	// #      #
	// #      #
	// #     #
	// ######
	// #     #
	// #      #
	// #      #
	// #      #
	// #       #
	//
	//
	//
	//
	0xFF, 0x41, 0x41, 0x41, 0x41, 0x41, 0xA2, 0x1C, 0x00,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x08,

	// @426 'S' (9 pixels wide)
	//   #####
	//  #     #
	// #       #
	// #
	//  #
	//   #####
	//        #
	//         #
	//         #
	// #       #
	//  #     #
	//   #####
	//
	//
	//
	//
	0x0C, 0x12, 0x21, 0x21, 0x21, 0x21, 0x21, 0x42, 0x84,
	0x02, 0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @444 'T' (9 pixels wide)
	// #########
	//     #
	//     #
	//     #
	//     #
	//     #
	//     #
	//     #
	//     #
	//     #
	//     #
	//     #
	//
	//
	//
	//
	0x01, 0x01, 0x01, 0x01, 0xFF, 0x01, 0x01, 0x01, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,

	// @462 'U' (8 pixels wide)
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	// #      #
	//  #    #
	//   ####
	//
	//
	//
	//
	0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @478 'V' (11 pixels wide)
	// #         #
	// #         #
	//  #       #
	//  #       #
	//   #     #
	//   #     #
	//    #   #
	//    #   #
	//     # #
	//     # #
	//      #
	//      #
	//
	//
	//
	//
	0x03, 0x0C, 0x30, 0xC0, 0x00, 0x00, 0x00, 0xC0, 0x30, 0x0C, 0x03,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x0C, 0x03, 0x00, 0x00, 0x00, 0x00,

	// @500 'W' (15 pixels wide)
	// #             #
	// #      #      #
	//  #     #     #
	//  #     #     #
	//  #    # #    #
	//   #   # #   #
	//   #   # #   #
	//    # #   # #
	//    # #   # #
	//     #     #
	//     #     #
	//     #     #
	//
	//
	//
	//
	0x03, 0x1C, 0x60, 0x80, 0x00, 0x80, 0x70, 0x0E, 0x70, 0x80, 0x00, 0x80, 0x60, 0x1C, 0x03,
	0x00, 0x00, 0x00, 0x01, 0x0E, 0x01, 0x00, 0x00, 0x00, 0x01, 0x0E, 0x01, 0x00, 0x00, 0x00,

	// @530 'X' (11 pixels wide)
	// #         #
	//  #       #
	//   #     #
	//    #   #
	//     # #
	//      #
	//      #
	//     # #
	//    #   #
	//   #     #
	//  #       #
	// #         #
	//
	//
	//
	//
	0x01, 0x02, 0x04, 0x08, 0x90, 0x60, 0x90, 0x08, 0x04, 0x02, 0x01,
	0x08, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08,

	// @552 'Y' (11 pixels wide)
	// #         #
	//  #       #
	//   #     #
	//    #   #
	//     # #
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	//      #
	//
	//
	//
	//
	0x01, 0x02, 0x04, 0x08, 0x10, 0xE0, 0x10, 0x08, 0x04, 0x02, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,

	// @574 'Z' (8 pixels wide)
	// ########
	//        #
	//       #
	//      #
	//      #
	//     #
	//    #
	//   #
	//   #
	//  #
	// #
	// ########
	//
	//
	//
	//
	0x01, 0x01, 0x81, 0x41, 0x21, 0x19, 0x05, 0x03,
	0x0C, 0x0A, 0x09, 0x08, 0x08, 0x08, 0x08, 0x08,

	// @590 'a' (8 pixels wide)
	//
	//
	//
	//  #####
	// #     #
	//       #
	//       #
	//  ######
	// #     #
	// #     #
	// #     #
	//  ##### #
	//
	//
	//
	//
	0x10, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF0, 0x00,
	0x07, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x08,

	// @606 'b' (7 pixels wide)
	// #
	// #
	// #
	// # ###
	// ##   #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// ##   #
	// # ###
	//
	//
	//
	//
	0xFF, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0,
	0x0F, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @620 'c' (7 pixels wide)
	//
	//
	//
	//   ###
	//  #   #
	// #     #
	// #
	// #
	// #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0x20,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02,

	// @634 'd' (7 pixels wide)
	//       #
	//       #
	//       #
	//   ### #
	//  #   ##
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//
	//
	//
	//
	0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xFF,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x0F,

	// @648 'e' (7 pixels wide)
	//
	//
	//
	//   ###
	//  #   #
	// #     #
	// #     #
	// #######
	// #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0xE0, 0x90, 0x88, 0x88, 0x88, 0x90, 0xE0,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02,

	// @662 'f' (5 pixels wide)
	//    ##
	//   #
	//   #
	// #####
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//
	//
	//
	//
	0x08, 0x08, 0xFE, 0x09, 0x09,
	0x00, 0x00, 0x0F, 0x00, 0x00,

	// @672 'g' (7 pixels wide)
	//
	//
	//
	//   ### #
	//  #   ##
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//       #
	//       #
	//      #
	//  ####
	0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xF8,
	0x03, 0x84, 0x88, 0x88, 0x88, 0x44, 0x3F,

	// @686 'h' (7 pixels wide)
	// #
	// #
	// #
	// # ###
	// ##   #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//
	//
	//
	//
	0xFF, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,

	// @700 'i' (1 pixels wide)
	// #
	//
	//
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	//
	//
	//
	//
	0xF9,
	0x0F,

	// @702 'j' (2 pixels wide)
	//  #
	//
	//
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	//  #
	// #
	0x00, 0xF9,
	0x80, 0x7F,

	// @706 'k' (7 pixels wide)
	// #
	// #
	// #
	// #    #
	// #   #
	// #  #
	// # #
	// ###
	// #  #
	// #   #
	// #    #
	// #     #
	//
	//
	//
	//
	0xFF, 0x80, 0xC0, 0x20, 0x10, 0x08, 0x00,
	0x0F, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08,

	// @720 'l' (1 pixels wide)
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	//
	//
	//
	//
	0xFF,
	0x0F,

	// @722 'm' (11 pixels wide)
	//
	//
	//
	// # ##   ##
	// ##  # #  #
	// #    #    #
	// #    #    #
	// #    #    #
	// #    #    #
	// #    #    #
	// #    #    #
	// #    #    #
	//
	//
	//
	//
	0xF8, 0x10, 0x08, 0x08, 0x10, 0xE0, 0x10, 0x08, 0x08, 0x10, 0xE0,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0F,

	// @744 'n' (7 pixels wide)
	//
	//
	//
	// # ###
	// ##   #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//
	//
	//
	//
	0xF8, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0,
	0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,

	// @758 'o' (7 pixels wide)
	//
	//
	//
	//   ###
	//  #   #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   #
	//   ###
	//
	//
	//
	//
	0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @772 'p' (7 pixels wide)
	//
	//
	//
	// # ###
	// ##   #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// ##   #
	// # ###
	// #
	// #
	// #
	// #
	0xF8, 0x10, 0x08, 0x08, 0x08, 0x10, 0xE0,
	0xFF, 0x04, 0x08, 0x08, 0x08, 0x04, 0x03,

	// @786 'q' (7 pixels wide)
	//
	//
	//
	//   ### #
	//  #   ##
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//       #
	//       #
	//       #
	//       #
	0xE0, 0x10, 0x08, 0x08, 0x08, 0x10, 0xF8,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0xFF,

	// @800 'r' (4 pixels wide)
	//
	//
	//
	// # ##
	// ##
	// #
	// #
	// #
	// #
	// #
	// #
	// #
	//
	//
	//
	//
	0xF8, 0x10, 0x08, 0x08,
	0x0F, 0x00, 0x00, 0x00,

	// @808 's' (6 pixels wide)
	//
	//
	//
	//  ####
	// #    #
	// #
	// #
	//  ####
	//      #
	//      #
	// #    #
	//  ####
	//
	//
	//
	//
	0x70, 0x88, 0x88, 0x88, 0x88, 0x10,
	0x04, 0x08, 0x08, 0x08, 0x08, 0x07,

	// @820 't' (5 pixels wide)
	//   #
	//   #
	//   #
	// #####
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//   #
	//    #
	//
	//
	//
	//
	0x08, 0x08, 0xFF, 0x08, 0x08,
	0x00, 0x00, 0x07, 0x08, 0x00,

	// @830 'u' (7 pixels wide)
	//
	//
	//
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	// #     #
	//  #   ##
	//   ### #
	//
	//
	//
	//
	0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8,
	0x03, 0x04, 0x08, 0x08, 0x08, 0x04, 0x0F,

	// @844 'v' (7 pixels wide)
	//
	//
	//
	// #     #
	// #     #
	//  #   #
	//  #   #
	//   # #
	//   # #
	//    #
	//    #
	//    #
	//
	//
	//
	//
	0x18, 0x60, 0x80, 0x00, 0x80, 0x60, 0x18,
	0x00, 0x00, 0x01, 0x0E, 0x01, 0x00, 0x00,

	// @858 'w' (11 pixels wide)
	//
	//
	//
	// #    #    #
	// #    #    #
	// #    #    #
	//  #  # #  #
	//  #  # #  #
	//  # #   # #
	//   ##   ##
	//   #     #
	//   #     #
	//
	//
	//
	//
	0x38, 0xC0, 0x00, 0x00, 0xC0, 0x38, 0xC0, 0x00, 0x00, 0xC0, 0x38,
	0x00, 0x01, 0x0E, 0x03, 0x00, 0x00, 0x00, 0x03, 0x0E, 0x01, 0x00,

	// @880 'x' (7 pixels wide)
	//
	//
	//
	// #     #
	//  #   #
	//  #   #
	//   # #
	//    #
	//   # #
	//  #   #
	//  #   #
	// #     #
	//
	//
	//
	//
	0x08, 0x30, 0x40, 0x80, 0x40, 0x30, 0x08,
	0x08, 0x06, 0x01, 0x00, 0x01, 0x06, 0x08,

	// @894 'y' (7 pixels wide)
	//
	//
	//
	// #     #
	// #     #
	//  #   #
	//  #   #
	//   # #
	//   # #
	//    #
	//    #
	//    #
	//   #
	//   #
	//   #
	// ##
	0x18, 0x60, 0x80, 0x00, 0x80, 0x60, 0x18,
	0x80, 0x80, 0x71, 0x0E, 0x01, 0x00, 0x00,

	// @908 'z' (6 pixels wide)
	//
	//
	//
	// ######
	//      #
	//     #
	//    #
	//   #
	//   #
	//  #
	// #
	// ######
	//
	//
	//
	//
	0x08, 0x08, 0x88, 0x48, 0x28, 0x18,
	0x0C, 0x0A, 0x09, 0x08, 0x08, 0x08,
};

// Character descriptors for Microsoft Sans Serif 12pt
// { [Char width in bits], [Offset into microsoftSansSerif_12ptCharBitmaps in bytes] }
const FONT_CHAR_INFO microsoftSansSerif_12ptDescriptors[] =
{
	{7, 0}, 		// 0
	{3, 14}, 		// 1
	{7, 20}, 		// 2
	{7, 34}, 		// 3
	{7, 48}, 		// 4
	{7, 62}, 		// 5
	{7, 76}, 		// 6
	{7, 90}, 		// 7
	{7, 104}, 		// 8
	{7, 118}, 		// 9
	{0, 0}, 		// :
	{0, 0}, 		// ;
	{0, 0}, 		// <
	{0, 0}, 		// =
	{0, 0}, 		// >
	{0, 0}, 		// ?
	{0, 0}, 		// @
	{11, 132}, 		// A
	{8, 154}, 		// B
	{10, 170}, 		// C
	{9, 190}, 		// D
	{8, 208}, 		// E
	{8, 224}, 		// F
	{10, 240}, 		// G
	{8, 260}, 		// H
	{1, 276}, 		// I
	{6, 278}, 		// J
	{8, 290}, 		// K
	{7, 306}, 		// L
	{9, 320}, 		// M
	{7, 338}, 		// N
	{10, 352}, 		// O
	{8, 372}, 		// P
	{10, 388}, 		// Q
	{9, 408}, 		// R
	{9, 426}, 		// S
	{9, 444}, 		// T
	{8, 462}, 		// U
	{11, 478}, 		// V
	{15, 500}, 		// W
	{11, 530}, 		// X
	{11, 552}, 		// Y
	{8, 574}, 		// Z
	{0, 0}, 		// [
	{0, 0}, 		// \
	{0, 0}, 		// ]
	{0, 0}, 		// ^
	{0, 0}, 		// _
	{0, 0}, 		// `
	{8, 590}, 		// a
	{7, 606}, 		// b
	{7, 620}, 		// c
	{7, 634}, 		// d
	{7, 648}, 		// e
	{5, 662}, 		// f
	{7, 672}, 		// g
	{7, 686}, 		// h
	{1, 700}, 		// i
	{2, 702}, 		// j
	{7, 706}, 		// k
	{1, 720}, 		// l
	{11, 722}, 		// m
	{7, 744}, 		// n
	{7, 758}, 		// o
	{7, 772}, 		// p
	{7, 786}, 		// q
	{4, 800}, 		// r
	{6, 808}, 		// s
	{5, 820}, 		// t
	{7, 830}, 		// u
	{7, 844}, 		// v
	{11, 858}, 		// w
	{7, 880}, 		// x
	{7, 894}, 		// y
	{6, 908}, 		// z
};

// Font information for Microsoft Sans Serif 12pt
const FONT_INFO microsoftSansSerif_12ptFontInfo =
{
	2, //  Character height
	'0', //  Start character
	'z', //  End character
	2, //  Width, in pixels, of space character
	microsoftSansSerif_12ptDescriptors, //  Character descriptor array
	microsoftSansSerif_12ptBitmaps, //  Character bitmap array
};

