#ifndef _GR_LIB_H_
#define _GR_LIB_H_

#define MAXCOLORS              16
#define SCREEN_WIDTH          256
#define SCREEN_HEIGHT          64
#define MAXSIZECHAR			   32

typedef unsigned short	zz_color_t;
typedef signed short	zz_coord_t;
typedef unsigned short	zz_size_t;
typedef unsigned int	zz_style_t;
typedef unsigned int	zz_type_t;

struct OLED_bitmap
{
   unsigned short bmpWidth;
   unsigned short bmpHeight;
   char           BitmapData[0];
};

/*
    Вывод пикселя на экран пиксел
    Параметры:
    x - горизотальная координата точки
    y - вертикальная координата точки
    color - цвет точки
*/
void OLED_putPixel(zz_coord_t x, zz_coord_t y, zz_color_t color);

/*
    Вывод битмапа на экран
    Параметры:
    x - горизонтальная координата верхнего левого угла битмапа
    y - вертикальная координата верхнего левого угла битмапа
    bmp - указатель на битмап
*/
void OLED_putBitmap(zz_coord_t x, zz_coord_t y, const struct OLED_bitmap* bmp);

#endif
