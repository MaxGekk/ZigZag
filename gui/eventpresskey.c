#include <string.h>
#include "scrlib.h"
#include "grlib.h"


const struct zzCodeTable* GetTable(struct zzTextField* textField, enum zzMessages msg)
{
	int i=0;
	
	while(textField->tables[i] != 0)
	{
		if(textField->tables[i]->code == msg)
		{
			if(textField->carriage->msgcode != msg)
			{
				textField->carriage->msgcode = msg;
				textField->carriage->active = 0;
			}
			return textField->tables[i];
		}
		i++;
	}
	return 0;
}


char GetChar(struct zzTextField* textField, enum zzMessages msg)
{

	const struct zzCodeTable* table;
	unsigned short position = GetCodeTablePositon();
	
		
	table = GetTable(textField, msg);
	if(table == 0)
	{
		textField->carriage->msgcode = -1;
		return -1;
	}
	if(textField->carriage->active == 0)
		position = 0;
	else
	{
		position++;
		if(position >= table->size)
			position = 0;
			
	}
	SetCodeTablePositon(position);
	return table->codeTable[position];
}


void EventTextFieldPressKey(struct zzTextField* textField, enum zzMessages msg)
{
	char buff[128];
	int flg,ptr = 0;

	if(msg != KEY_BACKSPACE)
	{
		textField->carriage->code = GetChar(textField,msg);

		if(textField->carriage->msgcode == -1)
			return;

		if(textField->carriage->active == 0)
		{
			strncpy(buff, textField->link, textField->carriage->txtposition);
			sprintf(buff + textField->carriage->txtposition, "%c", textField->carriage->code);
			sprintf(buff + textField->carriage->txtposition + 1, "%s", textField->link + textField->carriage->txtposition);
			textField->carriage->active = 1;
			ptr = 1;
		}
		else
		{
			
			strncpy(buff, textField->link, textField->carriage->txtposition - 1);
			sprintf(buff + textField->carriage->txtposition - 1, "%c", textField->carriage->code);
			sprintf(buff + textField->carriage->txtposition, "%s", textField->link + textField->carriage->txtposition);
			ptr = 0;
		}	
	}
	else
	{
		textField->carriage->msgcode = KEY_BACKSPACE;
		textField->carriage->active = 0;
		if(textField->carriage->txtposition > 0)
		{
			textField->carriage->code = textField->link[textField->carriage->txtposition - 1];
			strncpy(buff, textField->link, textField->carriage->txtposition - 1);
			sprintf(buff + textField->carriage->txtposition - 1, "%s", textField->link + textField->carriage->txtposition);
			ptr = -1;
		}
		else
		{
			textField->carriage->code = -1;
			strcpy(buff, textField->link);
			ptr = 0;
		}
	}
	zzSetTextFieldText(textField, buff);
	

	ChangeCarriageTextPosition(textField, ptr);
	ChangeCarraigeTextFieldPositon(textField, ptr);
	
	if(ptr != 0)
		DrawTextField(textField);
	if(msg == KEY_BACKSPACE)
		StartCarriage();
	DrawCarriageTextField(textField, CARRIAGEPUT);
	if(msg == KEY_BACKSPACE)
		StartCarriage();
}

void EventPressKey(struct zzControl* ctrl, enum zzMessages msg)
{
	if(ctrl != 0 &&  (GetControlState(ctrl) & STATE_FOCUSED) != 0)
	{
		switch(ctrl->type)
		{
			case TEXTFIELD:
				if(msg == KEY_BACKSPACE || msg>=KEY_ZERO && msg<=KEY_NINE)
					EventTextFieldPressKey((struct zzTextField*)ctrl, msg);

				if(((struct zzTextField*)ctrl)->OnPressKey != 0)
					(*((struct zzTextField*)ctrl)->OnPressKey)(ctrl,msg,0);

				break;
			default:
				break;

		}
	}
}
