#include <assert.h>
#include <string.h>
#include "scrlib.h"
#include "grlib.h"



void DrawControl(const struct zzControl* ctrl);


/*
    Нарисовать линию
    алгоритм Брезенхема генерации 8-ми связной развертки отрезка.
    Взято с http://alglib.sources.ru/graphics/section8connected.php
    
    Параметры:
    x1 - начальная горизонтальная координата 
    y1 - начальная вертикальная координата
    x2 - конечная горизонтальная координата 
    y2 - конечная вертикальная координата
    color - цвет линии
*/
void DrawLine
   (zz_coord_t x1, 
    zz_coord_t y1, 
    zz_coord_t x2, 
    zz_coord_t y2, 
    zz_color_t color)
{

    int x;
    int y;
    int dx;
    int dy;
    int sx;
    int sy;
    int z;
    int e;
    int i;
    int ch;

    x = x1;
    y = y1;
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = (x2 - x1 > 0) ? 1 : -1;
    sy = (y2 - y1 > 0) ? 1 : -1;
    
    if( dx == 0 && dy == 0 )
    {
        OLED_putPixel(x, y, color);
        return;
    }
    if( dy > dx )
    {
        z = dx;
        dx = dy;
        dy = z;
        ch = 1;
    }
    else
    {
        ch = 0;
    }
    e = 2 * dy-dx;
    i = 1;
    do
    {
        OLED_putPixel(x, y, color);
        while(e>=0)
        {
            if( ch )
            {
                x = x + sx;
            }
            else
            {
                y = y + sy;
            }
            e = e - 2 * dx;
        }
        if( ch )
        {
            y = y + sy;
        }
        else
        {
            x = x + sx;
        }
        e = e + 2 * dy;
        i = i + 1;
    }
    while( i <= dx);
    OLED_putPixel(x, y, color);
}

/*
    Нарисовать прямоугольник
    Параметры:
    x - горизонтальная координата верхнего левого угла
    y - вертикальная координата верхнего левого угла
    width - размер области по оси X
    height - размер области по оси Y
    color - цвет линии
*/
void DrawRectangle
   (zz_coord_t x, 
    zz_coord_t y, 
    zz_coord_t width, 
    zz_coord_t height, 
    zz_color_t color)
{
	int i;
    DrawLine(x, y, x + width, y, color);
	for(i = 1; i < height; i++)
	{
		OLED_putPixel(x, y + i, color);
		OLED_putPixel(x + width, y + i, color);
	}
    DrawLine(x, y + height, x + width, y + height, color);
}

/*
    Нарисовать залитый прямоугольник
    Параметры:
    x - горизонтальная координата верхнего левого угла
    y - вертикальная координата верхнего левого угла
    width - размер области по оси X
    height - размер области по оси Y
    color - цвет линии & заливки
*/
void DrawFilledRectangle
   (zz_coord_t x,
    zz_coord_t y,
    zz_coord_t width,
    zz_coord_t height,
    zz_color_t color)
{
    int j;
    for( j = y; j <= y + height; ++j ) 
		DrawLine(x, j, x + width, j, color);
}


/*
 * Расчет пересечения двух прямоугольников
 * Параметры:
 *    rect1        первый прямоугольник
 *    rect2        второй прямоугольник
 *    rect         Результат пересечения двух прямоугольников
 * Возвращаемое значение:
 *    Если пересечение пустое, то 0, иначе не нуль
 *    
 */
int IntersectRectangles
   (const struct rectangle* rect1,
    const struct rectangle* rect2,
    struct rectangle* rect)
{
    assert(rect1 != 0);
    assert(rect2 != 0);
    assert(rect != 0);
    
    if ( rect1 == 0 || rect2 == 0 || rect == 0 )
        return -1;

    rect->x = 0;
    rect->y = 0;
    rect->height = 0;
    rect->width = 0;


    if ( rect1->x > rect2->x + rect2->width
            || rect1->x + rect1->width < rect2->x
            || rect1->y + rect1->height < rect2->y
            || rect2->y + rect2->height < rect1->y ) {
        return 0;
    }
    else
    {
        rect->y = (rect1->y - rect2->y >= 0) ? rect1->y : rect2->y;
        rect->x = (rect1->x - rect2->x >= 0) ? rect1->x : rect2->x;

        if ( rect1->x <= rect2->x + rect2->width
                && rect1->x + rect1->width >= rect2->x + rect2->width ) {
            rect->width = rect2->x + rect2->width - rect->x;
        } else {
            rect->width = rect1->x + rect1->width - rect->x;
        }
        
        if ( rect1->y <= rect2->y + rect2->height
                && rect1->y + rect1->height >= rect2->y + rect2->height ) {
            rect->height = rect2->y + rect2->height - rect->y;
        } else {
            rect->height = rect1->y + rect1->height - rect->y;
        }
    }
    return 1;
}

void DrawBoundary(const struct zzControl* ctrl, short state)
{
	zz_color_t clr1,clr2;

	if(ctrl->style & STYLE_NOBORDER)
		return;

	switch(ctrl->type)
	{
	case LABEL:
		DrawRectangle(ctrl->rect.x,
			ctrl->rect.y,
			ctrl->rect.width,
			ctrl->rect.height,
			8);

		break;
	case BUTTON:
		
		clr1 = ctrl->bgColor + ((state & STATE_CLICKED) ? - 2 : 2);
		clr2 = ctrl->bgColor + ((state & STATE_CLICKED) ? 2 : - 2);
		
		DrawLine(ctrl->rect.x, 
			ctrl->rect.y, 
			ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y, clr1);

		DrawLine(ctrl->rect.x, 
			ctrl->rect.y,
			ctrl->rect.x,
			ctrl->rect.y + ctrl->rect.height, clr1);

		DrawLine(ctrl->rect.x + 1,
			ctrl->rect.y + ctrl->rect.height,
			ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y + ctrl->rect.height, clr2);

		DrawLine(ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y + 1,
			ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y + ctrl->rect.height, clr2);

		break;
	case TEXTFIELD:
		clr1 = 8;
		clr2 = 12;
		
		DrawLine(ctrl->rect.x, 
			ctrl->rect.y, 
			ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y, clr1);

		DrawLine(ctrl->rect.x, 
			ctrl->rect.y,
			ctrl->rect.x,
			ctrl->rect.y + ctrl->rect.height, clr1);

		DrawLine(ctrl->rect.x + 1,
			ctrl->rect.y + ctrl->rect.height,
			ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y + ctrl->rect.height, clr2);

		DrawLine(ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y + 1,
			ctrl->rect.x + ctrl->rect.width,
			ctrl->rect.y + ctrl->rect.height, clr2);
		break;
	}
}




short GetXPositoinText(const struct zzControl* ctrl, const char* str)
{
	if(ctrl->style & STYLE_LEFT)
		return ctrl->rect.x;		
	else if(ctrl->style & STYLE_CENTER)
		return ctrl->rect.x + ctrl->rect.width/2 - (int)strlen(str) * ctrl->font->width/2;
	else if(ctrl->style & STYLE_RIGHT)
		return ctrl->rect.x + ctrl->rect.width - (int)strlen(str) * ctrl->font->width;
	else
	{
		switch(ctrl->type)
		{
		case LABEL:
			return ctrl->rect.x;
			break;
		case BUTTON:
			return ctrl->rect.x + ctrl->rect.width/2 - (int)strlen(str) * ctrl->font->width/2;
			break;
		default:
			return ctrl->rect.x;
			break;
		}
	}
}

short GetYPositoinText(const struct zzControl* ctrl, const char* str)
{
	if(ctrl->style & STYLE_TOP)
		return ctrl->rect.y;		
	else if(ctrl->style & STYLE_MIDDLE)
		return ctrl->rect.y + ctrl->rect.height/2 - ctrl->font->height/2;
	else if(ctrl->style & STYLE_BOTTOM)
		return ctrl->rect.y + ctrl->rect.height - ctrl->font->height;
	else
	{
		switch(ctrl->type)
		{
		case LABEL:
			return ctrl->rect.y;
			break;
		case BUTTON:
			return ctrl->rect.y + ctrl->rect.height/2 - ctrl->font->height/2;
			break;
		default:
			return ctrl->rect.y;
			break;
		}
	}
}

void DrawBackground(const struct zzControl* ctrl, short state)
{
	if((ctrl->style & STYLE_TRANSPARENT))
	{
		DrawFilledRectangle(ctrl->rect.x,
							ctrl->rect.y,
							ctrl->rect.width,
							ctrl->rect.height,
							0);

		return;
	}
		
		
	switch(ctrl->type)
	{
	case BUTTON:
		DrawFilledRectangle(ctrl->rect.x,
						ctrl->rect.y,
						ctrl->rect.width,
						ctrl->rect.height,
						(state & STATE_FOCUSED) ? ctrl->bgColor + 1 : ctrl->bgColor);
		break;
	case LABEL:	case TEXTFIELD:
		DrawFilledRectangle(ctrl->rect.x,
							ctrl->rect.y,
							ctrl->rect.width,
							ctrl->rect.height,
							ctrl->bgColor);
		break;

	}
	
}


void DrawButton(const struct zzButton* button) {

	short state = GetControlState(button);
	
	struct rectangle rect;

	DrawBackground(&button->ctrl, state);
	DrawBoundary(&button->ctrl, state);	

	if(button->link)
	{
		PutString(GetXPositoinText(&button->ctrl,button->link) + ((state & STATE_CLICKED) && (state & STATE_FOCUSED)),
				GetYPositoinText(&button->ctrl,button->link) + ((state & STATE_CLICKED)  && (state & STATE_FOCUSED)),
				button->ctrl.font,
				button->link,
				(button->ctrl.style & STYLE_TRANSPARENT) ? button->ctrl.color + (state & STATE_FOCUSED) : 0,
				button->ctrl.color,
				&button->ctrl.rect,
				FLG_TEXT);
	}
}


void DrawLabel(const struct zzLabel* label)
{
	short state = GetControlState(label);

	DrawBackground(&label->ctrl, state);
	DrawBoundary(&label->ctrl, state);	
	
	if(label->link)
	{
		PutString(GetXPositoinText(&label->ctrl, label->link) + 1,
						GetYPositoinText(&label->ctrl, label->link) + 1,
						label->ctrl.font,
						label->link,
						label->ctrl.color,
						label->ctrl.bgColor,
						&label->ctrl.rect,
						FLG_TEXT);
	
	}
}

void DrawTextField(const struct zzTextField* textField)
{
	unsigned short state = GetControlState(textField);

	DrawBackground(&textField->ctrl,state);
	DrawBoundary(&textField->ctrl, state);

	if(textField->link)
	{
		PutString(GetXPositoinText(&textField->ctrl, textField->link) + 1,
						GetYPositoinText(&textField->ctrl, textField->link) + 1,
						textField->ctrl.font,
						textField->link,
						textField->ctrl.color,
						textField->ctrl.bgColor,
						&textField->ctrl.rect,
						FLG_TEXT);
	
	}
	
}

void DrawControl(const struct zzControl* ctrl) {

	short state = GetControlState(ctrl);

	if(state == -1)
		return;

	switch ( ctrl->type ) {
    case BUTTON:
		DrawButton((struct zzButton*) ctrl);
        break;
	case LABEL:
		DrawLabel((struct zzLabel*) ctrl);
		break;
	case TEXTFIELD:
		DrawTextField((struct zzTextField*) ctrl);
		break;
    default:
        break;
    }
}
