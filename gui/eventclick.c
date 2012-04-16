#include <string.h>
#include <time.h>
#include "scrlib.h"
#include "grlib.h"


void EventClick(const struct zzControl* ctrl, enum zzMessages msg, long data)
{
	switch(ctrl->type)
	{
	case BUTTON:
		if(((struct zzButton*)ctrl)->OnClick != 0)
			(*((struct zzButton*)ctrl)->OnClick)(ctrl,msg,data);
	break;
	}
}

