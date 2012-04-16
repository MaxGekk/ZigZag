#include <assert.h>
#include <string.h>
#include "scrlib.h"
#include "grlib.h"



struct zzPrivateControl* GetFocus(void);


struct zzPrivateControl
{
	const struct zzControl*			ctrl;
	unsigned short				state;
	unsigned short				codeTabelIndex;
};

struct zzPrivateScreen
{
	unsigned short				id;
	struct zzPrivateControl*	ctrls;
	unsigned short				count;
	short						index;

	zzMSGHANDLER				onShow;
};

struct zzPrivateScreens
{
	struct zzPrivateScreen*		screens;
	unsigned short				count;
	short						index;
};

void SetFocus(struct zzPrivateControl* ctrl);
void DrawScreenControls(const struct zzPrivateScreen* scr);

static struct zzPrivateScreens	*g_Screens;


unsigned short GetCodeTablePositon()
{
	struct zzPrivateControl* ctrl = GetFocus();
	return ctrl->codeTabelIndex;
}



 void SetCodeTablePositon(unsigned short index)
 {
	struct zzPrivateControl* ctrl = GetFocus();
	ctrl->codeTabelIndex = index;
 }

 short GetControlState(const struct zzControl* ctrl)
 {
	unsigned short index;
	if(g_Screens != 0)
	{
		for(index = 0; index < g_Screens->screens[g_Screens->index].count; index++)
		{
			if(ctrl == g_Screens->screens[g_Screens->index].ctrls[index].ctrl)
				return g_Screens->screens[g_Screens->index].ctrls[index].state;
		}
	}
	else
		return -1;
 }

void SetControlState(const struct zzControl* ctrl, unsigned short state)
{
	unsigned short index;
	if(g_Screens != 0)
	{
		for(index = 0; index < g_Screens->screens[g_Screens->index].count; index++)
		{
			if(ctrl == g_Screens->screens[g_Screens->index].ctrls[index].ctrl)
			{
				g_Screens->screens[g_Screens->index].ctrls[index].state |= state;
				break;
			}
		}
	}
}

void KillControlState(const struct zzControl* ctrl, unsigned short state)
{
	unsigned short index;
	if(g_Screens != 0)
	{
		for(index = 0; index < g_Screens->screens[g_Screens->index].count; index++)
		{
			if(ctrl == g_Screens->screens[g_Screens->index].ctrls[index].ctrl)
			{
				g_Screens->screens[g_Screens->index].ctrls[index].state &= ~state;
				break;
			}
		}
	}
}

void InitFocusControl(const struct zzControl** ctrls, struct zzPrivateScreen* result)
{
	unsigned short index = 0;
	for (index = 0; ctrls[index] != 0; ++index) 
	{
        if (ctrls[index]->style & STYLE_FOCUSABLE)
		{
			result->index = index;
			SetFocus(&result->ctrls[index]);
            break;
        }
    }
    if (ctrls[index] == 0 )
		result->index = -1;
}

void InitScreen(const struct zzScreen* scr, struct zzPrivateScreen* result)
{
	unsigned short index = 0;

	for(index = 0; scr->ctrls[index] !=0; ++index);
	assert(index > 0);
	
	result->ctrls = (struct zzPrivateControl*) malloc(sizeof(struct zzPrivateControl) * index);


	result->id = scr->id;
	result->count = index;
	result->onShow = scr->onShow;

	for(index = 0; scr->ctrls[index] !=0; ++index)
	{
		result->ctrls[index].ctrl = scr->ctrls[index];
		result->ctrls[index].state = 0;
		result->ctrls[index].codeTabelIndex = 0;
	}
}


void zzInitScreens(const struct zzScreen **scrs)
{
	unsigned short index = 0;

	g_Screens = (struct zzPrivateScreens*) malloc(sizeof(struct zzPrivateScreens));

	for(index = 0; scrs[index] != 0; ++index);
	assert(index > 0);

	g_Screens->index = 0;
	g_Screens->count = index;
	g_Screens->screens = (struct zzPrivateScreen*) malloc(sizeof(struct zzPrivateScreens) * g_Screens->count);
	for(index = 0; scrs[index] != 0; index++)
	{
		InitScreen(scrs[index], &g_Screens->screens[index]);
	}
}


struct zzPrivateControl* GetFocus() {

	return &g_Screens->screens[g_Screens->index].ctrls[g_Screens->screens[g_Screens->index].index];
}


int KillFocus(struct zzPrivateControl* ctrl) {
	
	int result = 1;
	assert(ctrl->ctrl->style & STYLE_FOCUSABLE);
    assert(ctrl->state & STATE_FOCUSED);

	if ( ctrl->ctrl->KillFocus != 0 ) {
		result = (*ctrl->ctrl->KillFocus)(ctrl->ctrl, KILL_FOCUS, 0);
    }
	if(result)
	{
		ctrl->state &= ~STATE_FOCUSED;
		StopCarriage();
		DrawControl(ctrl->ctrl);
	}
	return result;

}

void SetFocus(struct zzPrivateControl* ctrl) {

	assert(ctrl->ctrl->style & STYLE_FOCUSABLE);
    assert(!(ctrl->state & STATE_FOCUSED));

	ctrl->state |= STATE_FOCUSED;

	if ( ctrl->ctrl->SetFocus != 0 ) {
		(*ctrl->ctrl->SetFocus)(ctrl->ctrl, SET_FOCUS, 0);
    }
	SetCarriage(ctrl->ctrl);
	DrawControl(ctrl->ctrl);
	StartCarriage();
}



const struct zzControl* zzGetFocus() {
	return g_Screens->screens[g_Screens->index].ctrls[g_Screens->screens[g_Screens->index].index].ctrl;
}





void FireKeyMessage(enum zzMessages msg) {
	struct zzPrivateControl* ctrl = GetFocus();

	if ( g_Screens->screens[g_Screens->index].onShow != 0 ) {
		if((*g_Screens->screens[g_Screens->index].onShow)(ctrl->ctrl, msg, 0) == 0)
			return;
	}

	EventPressKey(ctrl->ctrl, msg);

	if(msg == KEY_ENTER)
	{
		if(!(ctrl->state & STATE_FOCUSED))
			return;

		ctrl->state |= STATE_CLICKED;
		DrawControl(ctrl->ctrl);

		EventClick(ctrl->ctrl, msg, 0);

		ctrl->state &= ~STATE_CLICKED;
		DrawControl(ctrl->ctrl);
	}
}

void RollFocus() {
	int index, result = 1;
	struct zzPrivateControl* ctrl;

	if ( g_Screens->screens[g_Screens->index].index != -1 ) {
        result = KillFocus(GetFocus());
    }
	if(!result)
		return;

	for ( index = g_Screens->screens[g_Screens->index].index + 1; index != g_Screens->screens[g_Screens->index].index; ++index ) {
        ctrl = &g_Screens->screens[g_Screens->index].ctrls[index];
		if ( index >= g_Screens->screens[g_Screens->index].count ) { /* restart from the beginning */
            index = 0;
            ctrl = &g_Screens->screens[g_Screens->index].ctrls[index];
        }

		if ( ctrl->ctrl != 0 && (ctrl->ctrl->style & STYLE_FOCUSABLE) != 0 ) {
            g_Screens->screens[g_Screens->index].index = index;
            SetFocus(ctrl);
            break;
        }
    }
}

void FireShowScreen() {
	if ( g_Screens->screens[g_Screens->index].onShow != 0 ) {
		if((*g_Screens->screens[g_Screens->index].onShow)(0, SHOW_SCREEN, 0) == 0)
			return;
	}
}


void InitializeScrLib(void) {

	int index;

	zzInitScreens(scrDescribe());

	FireShowScreen();
    
	DrawScreenControls(&g_Screens->screens[g_Screens->index]);
    
	for (index = 0; index < g_Screens->screens[g_Screens->index].count; ++index) {
		if(g_Screens->screens[g_Screens->index].ctrls[index].ctrl->style & STYLE_FOCUSABLE)
		{
			g_Screens->screens[g_Screens->index].index = index;
			SetFocus(&g_Screens->screens[g_Screens->index].ctrls[index]);
			break;
		}
    }
}

void DrawScreenControls(const struct zzPrivateScreen* scr) {
    int i;

	for ( i = 0; i < scr->count; i++ ) {
		DrawControl(scr->ctrls[i].ctrl);
    }
}



void zzSetButtonText(const struct zzButton* btn, const char* str)
{
	strcpy(btn->link, str);
}

void zzSetLabelText(const struct zzLabel* lbl, const char* str)
{
	strcpy(lbl->link, str);
}

void zzSetTextFieldText(const struct zzTextField* txtfld, const char* str)
{
	strcpy(txtfld->link, str);
	if(strlen(txtfld->link) == 0 && (GetControlState(&txtfld->ctrl) & STATE_FOCUSED))
		SetTextFieldCarriage(txtfld);
}

void zzSetText(const struct zzControl* ctrl, const char* str)
{
	switch(ctrl->type)
	{
	case BUTTON:
		zzSetButtonText((const struct zzButton*)ctrl, str);
		break;
	case LABEL:
		zzSetLabelText((const struct zzLabel*)ctrl, str);
		break;
	case TEXTFIELD:
		zzSetTextFieldText((const struct zzTextField*)ctrl, str);
		break;
	}
	DrawControl(ctrl);
}


