#include "typedefs.h"
#include "biquad.h"

//Biquad Variables Set

//Biquad Koefficients Set
TBQStageKoef FiltKoef[]=
{
#ifndef FiltStage0
	#error "Not Defined Stage0"
#else
	FiltStage0,
#endif

#ifdef FiltStage1
	FiltStage1,
#endif

#ifdef FiltStage2
	FiltStage2,
#endif

#ifdef FiltStage3
	FiltStage3,
#endif

#ifdef FiltStage4
	FiltStage4,
#endif
};

float BQVarSet[(BQNum+1)*3];
void BiquadInit(void)
{
	uchar	i;
	uchar*	Ptr;
	i=sizeof(BQVarSet);
	Ptr=(uchar*)&BQVarSet;
	while(i--) *(Ptr++)=0;
}

void BiquadShift(float* BQSet, float* Sample)
{
	uchar i=(BQNum+1)*3-1;
	while(i)
	{
		*(BQSet+i)=*(BQSet+(i-1));
		i--;
	}
	*BQSet=*Sample;
}

//Y0=X0*A0+X1*A1+X2*A2+Y1*B0+Y2*B1
float BiquadCompute(TBQStageKoef* Koef, TBQStageVar* Var)
{
	Var->Y0=Var->X0*Koef->A0;
	Var->Y0+=Var->X1*Koef->A1;
	Var->Y0+=Var->X2*Koef->A2;
	Var->Y0+=Var->Y1*Koef->B0;
	Var->Y0+=Var->Y2*Koef->B1;
	return Var->Y0;
}

sint IIRFilter(uint Sample)
{
	float floatSample;
	uchar i=0;
	floatSample=(float)Sample;

	BiquadShift ((float*)&BQVarSet, &floatSample);
	while(i<BQNum)
	{
		floatSample=BiquadCompute((TBQStageKoef*)&(FiltKoef[i]), (TBQStageVar*)&BQVarSet[i*3]);
		i++;
	}
	floatSample*=BQH;
	return (sint)floatSample;
}
