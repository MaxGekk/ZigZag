#include <string.h>
#include "scrlib.h"
#include "grlib.h"


void SetCarriage(const struct zzControl* ctrl);
void DrawCarriage(const struct  zzControl* ctrl, enum zzStateCarriage state);
void StartCarriage(void);
void StopCarriage(void);



void DrawActiveCarriageTextField(const struct  zzTextField* textField)
{
	struct rectangle rect;
	char buff[2];

	if(textField->carriage->msgcode == KEY_BACKSPACE)
		return;

	rect.x = textField->carriage->rect.x + 1 + ((textField->carriage->active) ? textField->carriage->position : textField->carriage->activeposition) - textField->ctrl.font->width;
	rect.y = textField->carriage->rect.y;
	rect.height = textField->ctrl.font->height;
	rect.width = textField->ctrl.font->width;
	
	sprintf(buff,"%c", textField->carriage->code);
	
	if(textField->carriage->active)
	{
		textField->carriage->activeposition = textField->carriage->position;
		textField->carriage->activetxtposition = textField->carriage->txtposition;

		DrawFilledRectangle(rect.x - ((textField->ctrl.font->width & 1)? 1 : 0) ,
							rect.y,
							textField->ctrl.font->width,
							textField->carriage->rect.height,
							textField->carriage->color);

		PutString(rect.x - 1, 
				rect.y - 1,
				textField->ctrl.font,
				buff ,
				textField->ctrl.bgColor,
				textField->ctrl.color,
				&textField->ctrl.rect,
				FLG_TEXT);


	}
	else
	{
		DrawFilledRectangle(rect.x - ((textField->ctrl.font->width & 1)? 1 : 0) ,
							rect.y,
							textField->ctrl.font->width,
							textField->carriage->rect.height,
							textField->carriage->bgColor);

		PutString(rect.x - 1, 
				rect.y - 1,
				textField->ctrl.font,
				buff ,
				textField->ctrl.color,
				textField->ctrl.bgColor,
				&textField->ctrl.rect,
				FLG_TEXT);

	}
}


void DrawCarriageTextField(const struct zzTextField* textField, enum zzStateCarriage state)
{
	if(state == CARRIAGESTOP)
		textField->carriage->state = 0;
	if(state == CARRIAGESTART) {
		textField->carriage->state = 1;
		if(textField->carriage->active)
		{
			textField->carriage->active = 0;
			if(textField->carriage->activeposition != textField->carriage->position || 
				textField->carriage->activetxtposition == textField->carriage->txtposition)
				DrawActiveCarriageTextField(textField);
		}
	}
	if(state == CARRIAGECHANGE) {
		textField->carriage->state = (textField->carriage->state) ? 0 : 1;
		if(textField->carriage->active)
		{

			textField->carriage->active = 0;
			DrawActiveCarriageTextField(textField);
		}
		
	}
	if(state == CARRIAGEPUT) {
		textField->carriage->state = 1;
		DrawActiveCarriageTextField(textField);
	}
	
	DrawFilledRectangle(textField->carriage->rect.x + 1 + textField->carriage->position -
							((textField->ctrl.font->width & 1)? 1 : 0) ,
			textField->carriage->rect.y,
			textField->carriage->rect.width,
			textField->carriage->rect.height,
			(textField->carriage->state == 1) ? textField->carriage->color : textField->carriage->bgColor);
	
}


void DrawCarriage(const struct zzControl* ctrl, enum zzStateCarriage state)
{
	switch(ctrl->type)
	{
	case TEXTFIELD:
		DrawCarriageTextField((const struct zzTextField*) ctrl, state);
		break;
	default:
		break;
	}
}


void SetTextFieldCarriage(const struct zzTextField* textField)
{
	int fld_len = (textField->ctrl.rect.width - 1)/textField->ctrl.font->width;

	textField->carriage->bgColor = textField->ctrl.bgColor;

	textField->carriage->color = textField->ctrl.color;

	textField->carriage->txtposition = strlen(textField->link);
	if(textField->carriage->txtposition > fld_len)
	{
		textField->carriage->txtposition = 0;
		textField->carriage->position = 0;
	}
	else
		textField->carriage->position = textField->carriage->txtposition * textField->ctrl.font->width;

	textField->carriage->state = 1;
	textField->carriage->rect.x = textField->ctrl.rect.x + 1;
	textField->carriage->rect.y = textField->ctrl.rect.y + 2;
	textField->carriage->rect.width = 0;
	textField->carriage->rect.height = textField->ctrl.rect.height - 4;

	textField->carriage->active = 0;
	textField->carriage->activeposition = 0;
	textField->carriage->activetxtposition = 0;

}


void SetCarriage(const struct zzControl* ctrl)
{
	if(ctrl != 0 &&  (GetControlState(ctrl) & STATE_FOCUSED) != 0)
		switch(ctrl->type)
	{
	case TEXTFIELD:
		SetTextFieldCarriage((struct zzTextField*) ctrl);
		break;
	default:
		break;
	}
}


int ChangeCarraigeTextFieldPositon(struct zzTextField* textField,char shift)
{
	int str_scrlen, fld_len;

	str_scrlen = textField->carriage->position + shift * textField->ctrl.font->width; 
	fld_len = (textField->ctrl.rect.width - 1);

	if(str_scrlen < fld_len && strlen(textField->link) * textField->ctrl.font->width >= str_scrlen)
	{
		textField->carriage->position += shift * textField->ctrl.font->width;
		return 0;
	}
	else if(strlen(textField->link) * textField->ctrl.font->width >= str_scrlen)
	{
		return 1;
	}
	else if(textField->carriage->position == 0)
	{
		return 1;
	}
	return 0;
}

int ChangeCarriageTextPosition(struct  zzTextField* textField,char shift)
{
	if(textField->carriage->txtposition + (int)shift >= 0 && textField->carriage->txtposition + (int)shift <= strlen(textField->link))
	{
		textField->carriage->txtposition += shift;
		return 1;
	}
	else
		return 0;

}


void CarriageState()
{
	const struct zzControl* ctrl = zzGetFocus();
	if(ctrl != 0 &&  (GetControlState(ctrl) & STATE_FOCUSED) != 0)
		DrawCarriage(ctrl,CARRIAGECHANGE);
}

void StopCarriage()
{
	const struct zzControl* ctrl = zzGetFocus();
	if(ctrl != 0 &&  (GetControlState(ctrl) & STATE_FOCUSED) != 0)
		DrawCarriage(ctrl,CARRIAGESTOP);
}

void StartCarriage()
{
	const struct zzControl* ctrl = zzGetFocus();
	if(ctrl != 0 &&  (GetControlState(ctrl) & STATE_FOCUSED) != 0)
		DrawCarriage(ctrl,CARRIAGESTART);
}


void ChangeCarraigePositon(char shift)
{
	const struct zzControl* ctrl = zzGetFocus();
	if(ctrl != 0 &&  (GetControlState(ctrl) & STATE_FOCUSED) != 0)
	{
		StopCarriage();
		switch(ctrl->type)
		{
		case TEXTFIELD:
			if(ChangeCarriageTextPosition((struct zzTextField*)ctrl,shift))
				if(ChangeCarraigeTextFieldPositon((struct zzTextField*)ctrl,shift))
					DrawControl(ctrl);
			break;
		default:
			break;
		}
		StartCarriage();
	}
}
