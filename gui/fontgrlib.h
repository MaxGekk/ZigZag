#ifndef _FONT_GR_LIB_H_
#define _FONT_GR_LIB_H_

#define MAXSIZECYMBOL 32

struct font
{
	const char* name;
	unsigned short width;
	unsigned short height;
	unsigned short baseline;
	const char* symbols;
	const unsigned short* codes;

};

void PutStringBitmap(int x,int y,const struct font *fnt,const char* str,int color,const struct rectangle* rect);

struct rectangle FindStringRect(const char* str,const struct font *fnt);


#endif
