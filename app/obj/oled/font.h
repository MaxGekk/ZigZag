/*
 * font.h - interface file for font processing routines and data
 */

#ifndef _FONT_H_
#define _FONT_H_

#define FONT_NUM
#define getFont(num) fh[num]

struct fontHeader
{
	unsigned short height;
	unsigned short width;
	unsigned short block_num; // Number of blocks
	struct blockHeader
	{
		unsigned char  fisrtSymCode;
		unsigned char  blkSize;
	}bh[0];
};

extern const struct fontHeader * fh[FONT_NUM];

enum fonts
{
   font6x9 =  0,
   font8x12 = 1
};

#endif  // _FONT_H_

