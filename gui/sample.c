#include "scrlib.h"

extern struct zzFont bdf6x9;
extern struct zzFont bdf7x13;
char textbtn1[1024];
char textbtn2[1024];

char textlabel1[1024];
char textlabel2[1024];
char textlabel3[1024];
char textlabel4[1024];

char texttxtfld1[1024];
char texttxtfld2[1024];

struct zzCarriage	carriage;


int OnClickButton1(const struct zzControl* ctrl, enum zzMessages msg, long ptr);
int OnClickButton2(const struct zzControl* ctrl, enum zzMessages msg, long ptr);

int OnKillFocusTextField1(const struct zzControl* ctrl, enum zzMessages msg, long ptr);

int OnPressKeyTextField1(const struct zzControl* ctrl, enum zzMessages msg, long ptr);


int OnShow1(const struct zzControl* ctrl, enum zzMessages msg, long ptr);

const struct zzCodeTable myCodeTable1 = {
	KEY_ONE,
	4,
	0,
	{
		'1',
		'A',
		'B',
		'C'
	}
};
const struct zzCodeTable myCodeTable2 = {
	KEY_TWO,
	4,
	0,
	{
		'2',
		'D',
		'E',
		'F'
	}
};

const struct zzCodeTable myCodeTable3 = {
	KEY_THREE,
	4,
	0,
	{
		'3',
		'G',
		'H',
		'I'
	}
};
const struct zzCodeTable myCodeTable4 = {
	KEY_FOUR,
	4,
	0,
	{
		'4',
		'J',
		'K',
		'L'
	}
};
const struct zzCodeTable myCodeTable5 = {
	KEY_FIVE,
	4,
	0,
	{
		'5',
		'M',
		'N',
		'O'
	}
};
const struct zzCodeTable myCodeTable6 = {
	KEY_SIX,
	5,
	0,
	{
		'6',
		'P',
		'Q',
		'R',
		'S'
	}
};
const struct zzCodeTable myCodeTable7 = {
	KEY_SEVEN,
	4,
	0,
	{
		'7',
		'T',
		'U',
		'V'
	}
};
const struct zzCodeTable myCodeTable8 = {
	KEY_EIGHT,
	5,
	0,
	{
		'8',
		'W',
		'X',
		'Y',
		'Z'
	}
};

const struct zzCodeTable* myTables[] = {
	&myCodeTable1,
	&myCodeTable2,
	&myCodeTable3,
	&myCodeTable4,
	&myCodeTable5,
	&myCodeTable6,
	&myCodeTable7,
	&myCodeTable8,
	0
};

struct zzLabel myLabel1 = {
	{
		0,
		LABEL,
		STYLE_LEFT,
		12,                 /* main color */
		0,                 /* backgorund color */
		&bdf6x9,		/* font */
		{2, 3, 22, 9 },
		0,
		0,
	},
	textlabel1,
	1024,
};


struct zzLabel myLabel2 = {
	{
		1,
		LABEL,
		STYLE_LEFT|STYLE_NOBORDER,
		12,                 /* main color */
		0,                 /* backgorund color */
		&bdf6x9,		/* font */
		{2, 16, 46, 9 },
		0,
		0,
	},
	textlabel2,
	1024,
};


struct zzLabel myLabel3 = {
	{
		1,
		LABEL,
		STYLE_LEFT|STYLE_NOBORDER,
		12,                 /* main color */
		8,                 /* backgorund color */
		&bdf6x9,		/* font */
		{2,28, 251, 9 },
		0,
		0,
	},
	textlabel3,
	1024,
};


struct zzLabel myLabel4 = {
	{
		1,
		LABEL,
		STYLE_LEFT|STYLE_NOBORDER,
		12,                 /* main color */
		8,                 /* backgorund color */
		&bdf6x9,		/* font */
		{2,39, 251, 9 },
		0,
		0,
	},
	textlabel4,
	1024,
};




const struct zzTextField myTextField1 = {
 {
		3,              /* control id */
		TEXTFIELD,         /* control type */
        STYLE_FOCUSABLE,/* style */
		8,                 /* main color */
		0,                 /* backgorund color */
		&bdf6x9,		/* font */
        { 27, 2, 226, 11 }, /* rectangle */
        OnKillFocusTextField1,              /* KillFocus handler */
        0              /* SetFocus handler */
     },
	 myTables,
     texttxtfld1,
	 1024,
	 &carriage,
	 OnPressKeyTextField1
};

const struct zzTextField myTextField2 = {
 {
		4,              /* control id */
		TEXTFIELD,         /* control type */
        STYLE_FOCUSABLE,/* style */
		8,                 /* main color */
		0,                 /* backgorund color */
		&bdf6x9,		/* font */
        { 51, 15, 202, 11 }, /* rectangle */
        0,              /* KillFocus handler */
        0              /* SetFocus handler */
     },
	 myTables,
     texttxtfld2,
	 1024,
	 &carriage,
};


const struct zzButton myButton1 = {
     {
        0,              /* control id */
        BUTTON,         /* control type */
        STYLE_FOCUSABLE,/* style */
		8,                 /* main color */
		10,                 /* backgorund color */
		&bdf6x9,		/* font */
        { 2, 50, 20, 11}, /* rectangle */
        0,              /* KillFocus handler */
        0, /* SetFocus handler */
     },
     textbtn1,              /* label */
	 1024,
     OnClickButton1                 /* OnClick handler */
};

const struct zzButton myButton2 = {
     {
        0,              /* control id */
        BUTTON,         /* control type */
        STYLE_FOCUSABLE,/* style */
		8,                 /* main color */
		10,                 /* backgorund color */
		&bdf6x9,		/* font */
        { 208, 50, 45, 8}, /* rectangle */
        0,              /* KillFocus handler */
        0, /* SetFocus handler */
     },
     textbtn2,              /* label */
	 1024,
     OnClickButton2                 /* OnClick handler */
};


const struct zzControl* myMainScreenControls1[] = {
	
	(struct zzControl*) &myTextField1,
	(struct zzControl*) &myTextField2,
    (struct zzControl*) &myButton1,
	(struct zzControl*) &myButton2,
	(struct zzControl*) &myLabel1,
	(struct zzControl*) &myLabel2,
	(struct zzControl*) &myLabel3,
	(struct zzControl*) &myLabel4,
	0
};

const struct zzScreen myMainScreen1 = {
    0,                          /* screen id */
    myMainScreenControls1,      /* controls */
    OnShow1                     /* message handler */
};



const struct zzScreen* myScreens[] = { &myMainScreen1, 0 };

const struct zzScreen **scrDescribe(void)
{
	zzSetText((const struct zzControl*) &myTextField1,"");
	zzSetText((const struct zzControl*) &myTextField2,"");
	zzSetText((const struct zzControl*) &myButton1,"Ok");
	zzSetText((const struct zzControl*) &myButton2,"Cancel");
	zzSetText((const struct zzControl*) &myLabel1,"тхн:");
	zzSetText((const struct zzControl*) &myLabel2,"рекетнм:");
	

    return myScreens;
}

int OnShow1(const struct zzControl* ctrl, enum zzMessages msg, long ptr)
{

	return 1;
}

int OnClickButton1(const struct zzControl* ctrl, enum zzMessages msg, long ptr)
{
	zzSetText((const struct zzControl*) &myLabel3,myTextField1.link);
	zzSetText((const struct zzControl*) &myLabel4,myTextField2.link);
	zzSetText((const struct zzControl*) &myTextField1,"");
	zzSetText((const struct zzControl*) &myTextField2,"");
}


int OnClickButton2(const struct zzControl* ctrl, enum zzMessages msg, long ptr)
{
	zzSetText((const struct zzControl*) &myTextField1,"");
	zzSetText((const struct zzControl*) &myTextField2,"");
}

int OnKillFocusTextField1(const struct zzControl* ctrl, enum zzMessages msg, long ptr)
{

	if(strlen(((struct zzTextField*)ctrl)->link) == 0)
		return 0;
	return 1;
}

int OnPressKeyTextField1(const struct zzControl* ctrl, enum zzMessages msg, long ptr)
{
	if(msg == KEY_ENTER)
		OnClickButton1((const struct zzControl*) &myButton1, msg, ptr);
}
