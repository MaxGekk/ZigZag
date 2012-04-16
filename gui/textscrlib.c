#include <string.h>
#include "scrlib.h"
#include "grlib.h"

extern const unsigned int cp1251[510];




int FindStringRect
	(const char* str,
	 const struct zzFont *fnt,
	 struct rectangle* result)
{
	int length = (int)strlen(str), j = 0, k = 1, i = 0;
	
	if(length <= 0)
		return 0;

	for(i=0; i < length; i++)
	{
		if(str[i] == '\n')
			k++;
		else
			j++;
	}
	result->x = 0;
	result->y = 0;
	
	result->height = k * fnt->height;
	result->width = j * fnt->width;

	return 1;
}

int FindCharRectangle
	(zz_coord_t					x,
	 zz_coord_t					y,
	 zz_coord_t					fWidth,
	 zz_coord_t					fHeight,
	 const struct rectangle*	rect,
	 struct rectangle*			result)
{
	
	if(rect == 0)
	{
		result->x = 0;
		result->y = 0;
		result->height = fHeight;
		result->width = fWidth;
	}
	else
	{

		if(x + fWidth < rect->x || y + fHeight < rect->y || 
			x > rect->x + rect->width || 
			y > rect->y + rect->height)
			return 0;

		result->y = (rect->y - y <= 0) ? 0 : rect->y - y;
		result->x = (rect->x - x <= 0) ? 0 : rect->x - x;

		result->height = (fHeight - result->y >= rect->height) ? rect->height : fHeight-result->y;
		result->width = (fWidth - result->x >= rect->width) ? rect->width : fWidth-result->x;
	}

	return 1;

}

short GetCharCode
	(const struct zzFont*	fnt,
	 char					code)
{
	int i=0,cd = (unsigned char)code;
	const struct zzCodesBlock* block;
	if(fnt->charset == CP1251)
	{
		cd = cp1251[2*cd+1];
	}
	while(fnt->blocks[i]!=0)
	{
		block=fnt->blocks[i];
		if(cd >= block->firstChar && cd < block->numChars + block->firstChar)
		{
			cd = cd - block->firstChar + block->indexChar;
			return  cd;
		}
		i++;
	}
	return -1;
}

void PutStringBitmap
	(zz_coord_t					x,
	 zz_coord_t					y,
	 const struct zzFont		*fnt,
	 const char*				str,
	 int						color,
	 int						bgColor,
	 const struct rectangle*	rect)
{
	int i=0, j=0, k=0, n=0, length = (int)strlen(str), clr;
	struct OLED_bitmap* bmp;
	char buffBitmap[sizeof(struct OLED_bitmap) + SCREEN_WIDTH];
	int src_stride, w_block,indexOfKey, block;


	struct rectangle rectStr,rectResult;

	bmp = (struct OLED_bitmap*)buffBitmap;

	src_stride = (fnt->width >> 1) + (fnt->width & 1 ? 1 : 0);
	w_block = fnt->width + ((fnt->width >> 2) ? 4 - fnt->width % 4 : 0);

	FindStringRect(str,fnt,&rectStr);
	rectStr.x = x;
	rectStr.y = y;
	IntersectRectangles(&rectStr,rect,&rectResult);

	bmp->bmpWidth = rectResult.width;
	bmp->bmpHeight = 1;

	for(i = 0; i < fnt->height; i++)
	{
		
		block = 0;
		for(j = 0; j < length; j++)
		{
			if(j > 0)
			block += src_stride - ((fnt->width * j & 1 ) ? 1 : 0);
			indexOfKey = GetCharCode(fnt, str[j]);
			k = 0;
			clr = 0;
			for(n = w_block - 1; n >= w_block - fnt->width; n--)
			{
				clr += ((fnt->chars[i + fnt->height * indexOfKey]>>n) & 1) ? color : bgColor;
				if(fnt->width * j & 1)
				{
					if(n == w_block - 1)
					{
						bmp->BitmapData[block] += clr;
						k++;
						clr = 0;
					}
					else
					{
						if(n & 1)
						{
							bmp->BitmapData[block + k] = clr;
							k++;
							clr = 0;
						}
						else
						{
							clr = clr << 4;
						}
					}
				}
				else
				{
					if(n & 1)
					{
						clr = clr << 4;
					}
					else
					{
						bmp->BitmapData[block + k] = clr;
						k++;
						clr = 0;
					}
					if(n == w_block - fnt->width && fnt->width & 1)
						bmp->BitmapData[block + k] = clr;
				}
			}
		}
		OLED_putBitmap(x, y + i, bmp);
	}	
}

void PutStringPixel
	(zz_coord_t					x,
	 zz_coord_t					y,
	 const struct zzFont		*fnt,
	 const char*				str,
	 int						color,
	 const struct rectangle*	rect)
{
	int i=0, j=0, k=0, n=0, length; 
	int w_block,indexOfKey;

	struct rectangle rectStr,rectResult;

	

	if(str != 0)
	{

	FindStringRect(str,fnt,&rectStr);
	rectStr.x = x;
	rectStr.y = y;
	IntersectRectangles(&rectStr,rect,&rectResult);

	length = (strlen(str) > rectResult.width / fnt->width) ? rectResult.width / fnt->width : strlen(str);
	w_block = fnt->width + ((fnt->width >> 2) ? 4 - fnt->width % 4 : 0);

	for(i = 0; i < fnt->height; i++)
	{
		for(j = 0; j < length; j++)
		{
			indexOfKey = GetCharCode(fnt, str[j]);

			for(n= w_block - fnt->width;n < w_block; n++)
			{
				if((fnt->chars[i + fnt->height * indexOfKey] >>(w_block - n +  (fnt->width & 1 ? 0 : 1))) & 1)
					OLED_putPixel(x + n +(fnt->width)*j,y+i,color);
			}
		}
	}
	}
}

void PutString
	(zz_coord_t					x,
	 zz_coord_t					y,
	 const struct zzFont		*fnt,
	 const char*				str,
	 int						color,
	 int						bgColor,
	 const struct rectangle*	rect,
	 enum zzDrawMode			mode)
{
	if(mode == FLG_BITMAP)
		PutStringBitmap(x, y, fnt, str, color, bgColor, rect);
	else
		PutStringPixel(x, y, fnt, str, color, rect);
		
}

