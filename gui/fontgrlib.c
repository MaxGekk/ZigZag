#include "grlib.h"
#include "drawgrlib.h"
#include "fontgrlib.h"

struct rectangle FindStringRect(const char* str,const struct font *fnt)
{
	struct rectangle rect;
	int length=strlen(str), j=1, k=1, i=0;

	for(i=0;i<length;i++)
	{
		if(str[i] == '\n')
			k++;
		else
			j++;
	}
	rect.x = 0;
	rect.y = 0;
	
	rect.height = k * fnt->height;
	rect.width = j * fnt->width;

	return rect;
}


void PutCharBitmap(unsigned short x,unsigned short y,const struct font *fnt,char symbol,unsigned short color,const struct rectangle* rect)
{
	struct OLED_bitmap* bmp;
	BYTE buffBitmap[sizeof(struct OLED_bitmap) + MAXSIZECYMBOL * MAXSIZECYMBOL];

	unsigned short i, j, k = 0, l = 0, indexOfKey, src_stride;
	unsigned short w_block;
	unsigned short clr;
	
	unsigned short rectOffsetY;
	unsigned short rectOffsetX;

	unsigned short rectHeight;
	unsigned short rectWidth;


	if(rect == NULL)
	{
		rectOffsetY = 0;
		rectOffsetX = 0;
		rectHeight = fnt->height;
		rectWidth = fnt->width;
	}
	else
	{

		if(x + fnt->width < rect->x || y + fnt->height < rect->y || 
			x > rect->x + rect->width || 
			y > rect->y + rect->height)
			return;

		rectOffsetY = (rect->y - y <= 0) ? 0 : rect->y - y;
		rectOffsetX = (rect->x - x <= 0) ? 0 : rect->x - x;

		rectHeight = (fnt->height - rectOffsetY >= rect->height) ? rect->height : fnt->height-rectOffsetY;
		rectWidth = (fnt->width - rectOffsetX >= rect->width) ? rect->width : fnt->width-rectOffsetX;
	}

	src_stride = (rectWidth >> 1) + (rectWidth & 1 ? 1 : 0);
	w_block = fnt->width + ((fnt->width >> 2) ? 4 - fnt->width % 4 : 0);
	
	bmp = (struct OLED_bitmap*)buffBitmap;
	bmp->bmpWidth = rectWidth;
	bmp->bmpHeight = rectHeight;

	indexOfKey = 0;
	while(symbol != fnt->codes[indexOfKey])
		indexOfKey++;

	for(i = rectOffsetY; i<rectOffsetY + rectHeight; i++)
	{
		k = 0;
		clr = 0;
		for(j = w_block - 1 - rectOffsetX; j >= w_block - 1 - rectOffsetX - rectWidth; j--)
		{
			clr += ((fnt->symbols[i + fnt->height * indexOfKey]>>j) & 1) ? color : 0;
			if((j + 1) & 1)
			{
				bmp->BitmapData[src_stride * (rectHeight - 1 - l) + k] = clr;
				k++;
				clr = 0;
			}
			else
				clr = clr << 4;
		}
		l++;
	}
	OLED_putBitmap(x + rectOffsetX, y + rectOffsetY, bmp);
}

void PutStringBitmap(int x,int y,const struct font *fnt,const char* str,int color,const struct rectangle* rect)
{
	int i, length = strlen(str);
	struct rectangle rc, rc1;

	for(i = 0; i < length; i++)
		PutCharBitmap(x + i * fnt->width ,y ,fnt ,str[i],color,rect);
}
