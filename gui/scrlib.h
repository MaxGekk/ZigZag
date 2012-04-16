#ifndef _SCREEN_LIB_H_
#define _SCREEN_LIB_H_

#include "grlib.h"

enum zzDrawMode {
	FLG_BITMAP,
	FLG_TEXT
};

struct rectangle
{
	zz_coord_t	x;
	zz_coord_t	y;
	zz_coord_t	width;
	zz_coord_t	height;
};


enum zzCharSet
{
	NAN,
	CP1251,
	KOI8R
};

struct zzCodesBlock
{
	unsigned short firstChar;
	unsigned short numChars;
	unsigned short indexChar;
};

struct zzFont
{
	const char*						name;
	enum zzCharSet					charset;
	zz_size_t						width;
	zz_size_t						height;
	zz_size_t						baseline;
	const char*						chars;
	const struct zzCodesBlock**		blocks;
};

enum zzMessages {
	KEY_UNKNOWN = -1,
    SHOW_SCREEN,
    KEY_UP,
    KEY_DOWN,
	KEY_ZERO,
	KEY_ONE,
	KEY_TWO,
	KEY_THREE,
	KEY_FOUR,
	KEY_FIVE,
	KEY_SIX,
	KEY_SEVEN,
	KEY_EIGHT,
	KEY_NINE,
	KEY_BACKSPACE,
	KEY_ENTER,
	KEY_ESCAPE,
    KILL_FOCUS,
    SET_FOCUS,
};

enum zzControlType {
	BUTTON,
	LABEL,
	TEXTFIELD
};


struct zzControl;
/**
 * Generic message handler
 *
 * Arguments:
 *      ctrl	active control
 *      msg     message ID, see zzMessages enumeration for possible values
 *      data    message dependent parameter, see description of the messages
 * Return value:
 *      message dependent, see description of the messages
 */
typedef int (*zzMSGHANDLER)
   (const struct zzControl*	ctrl,
    enum zzMessages			msg,
    long					data);


#define STYLE_FOCUSABLE		1
#define STYLE_NOBORDER		2
#define STYLE_TRANSPARENT	4

#define STYLE_RIGHT			8
#define STYLE_CENTER		16
#define STYLE_LEFT			32

#define STYLE_TOP			128
#define STYLE_MIDDLE		256
#define STYLE_BOTTOM		512

#define STATE_FOCUSED		1
#define STATE_CLICKED		2


struct zzControl
{
	unsigned short			id;
	enum zzControlType		type;
	zz_style_t				style;
	zz_color_t				color;
	zz_color_t				bgColor;
	struct zzFont*			font;
	struct rectangle		rect;

	/* Message handlers */
	zzMSGHANDLER			KillFocus;
	zzMSGHANDLER			SetFocus;
};

struct zzButton
{
	struct zzControl		ctrl;
	char*					link;
	zz_size_t				length;

	zzMSGHANDLER			OnClick;
};

struct zzLabel
{
	struct zzControl		ctrl;
	char*					link;
	zz_size_t				length;
	
	zzMSGHANDLER            OnChange;
};

enum zzStateCarriage {
	CARRIAGESTOP,
	CARRIAGECHANGE,
	CARRIAGESTART,
	CARRIAGEPUT
};

struct zzCarriage
{
	struct rectangle	rect;
	unsigned short		state;
	unsigned short		position;
	unsigned short		txtposition;
	unsigned short		active;
	unsigned short		activeposition;
	unsigned short		activetxtposition;
	zz_color_t			color;
	zz_color_t			bgColor;
	char				code;
	char				msgcode;
};

struct zzCodeTable
{
	char					code;
	zz_size_t				size;
	zz_coord_t				position;
	const char				codeTable[];
};

struct zzTextField
{
	struct zzControl			ctrl;
	const struct zzCodeTable**	tables;
	char*						link;
	zz_size_t					length;
	struct zzCarriage*			carriage;

	zzMSGHANDLER				OnPressKey;
};


/**
 * Screen description. A screen consists of a number of controls displayed
 * simultaneously.
 */
struct zzScreen
{
	unsigned short				id;     /* Screen ID */
	const struct zzControl**	ctrls;  /* Zero-terminated array of pointers to controls */
	
	zzMSGHANDLER			onShow;
};


/**
 * Call this function to switch to the given screen.
 * Arguments:
 *      scr             screen to activate
 * Return value:
 *      TODO
 */
int zzShowScreen(struct zzScreen* scr);


/**
 * Returns current control in focus
 * Arguments: none
 * Return value:
 *     pointer to the focused control, or null
 */
const struct zzControl* zzGetFocus();

void zzSetText(const struct zzControl* ctrl, const char* str);

void zzSetActiveScreen(const struct zzScreen* scr);

/**
 * Callback function to be implemented by the application which describes
 * the series of screens that will be displayed. The first screen in the array
 * is considered to be the default one and will be displayed first
 *
 * Arguments:
 *      none
 * Return value:
 *      array of pointers to the filled zzScreen structures
 */
extern const struct zzScreen **scrDescribe(void);



/*
    ���������� �����
    �������� ���������� ��������� 8-�� ������� ��������� �������.
    
    ���������:
    x1 - ��������� �������������� ���������� 
    y1 - ��������� ������������ ����������
    x2 - �������� �������������� ���������� 
    y2 - �������� ������������ ����������
    color - ���� �����
*/
void DrawLine
   (zz_coord_t	x1,
    zz_coord_t	y1,
    zz_coord_t	x2,
    zz_coord_t	y2,
    zz_color_t	color);

/*
    ���������� �������������
    ���������:
    x - �������������� ���������� �������� ������ ����
    y - ������������ ���������� �������� ������ ����
    width - ������ ������� �� ��� X
    height - ������ ������� �� ��� Y
    color - ���� �����
*/
void DrawRectangle
   (zz_coord_t	x,
    zz_coord_t	y,
    zz_coord_t	width,
    zz_coord_t	height,
    zz_color_t	color);

/*
    ���������� ������� �������������
    ���������:
    x - �������������� ���������� �������� ������ ����
    y - ������������ ���������� �������� ������ ����
    width - ������ ������� �� ��� X
    height - ������ ������� �� ��� Y
    color - ���� ����� & �������
*/
void DrawFilledRectangle
   (zz_coord_t	x,
    zz_coord_t	y,
    zz_coord_t	width,
    zz_coord_t	height,
    zz_color_t	color);

/*
 * ������ ����������� ���� ���������������
 * ���������:
 *    r1        ������ �������������
 *    r2        ������ �������������
 *    result    ��������� ����������� ���� ���������������
 * ������������ ��������:
 *    ���� ����������� ������, �� 0, 1 - ���� �����������, -1 ���� �������� ������� ���������
 *    
 */
int IntersectRectangles
   (const struct rectangle* r1,
    const struct rectangle* r2,
    struct rectangle*       result);

/*
	����� ������
	���������:
	x - �������������� ���������� �������� ������ ����
	y - ������������ ���������� �������� ������ ����	
	font - ��������� �� �������� ������
	color - ���� ������
	bgColor - ���� �������
	rect - �������������, � ������� ����������� ������
	mode - ������ ����������� ������ (zzDrawMode)
*/
void PutString
	(zz_coord_t					x,
	 zz_coord_t					y,
	 const struct zzFont		*font,
	 const char*				str,
	 int						color,
	 int						bgColor,
	 const struct rectangle		*rect,
	 enum zzDrawMode			mode);

#endif

