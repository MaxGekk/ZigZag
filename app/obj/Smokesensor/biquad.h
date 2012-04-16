#ifndef BIQUAD_H
#define BIQUAD_H
#include "typedefs.h"

//Y0=X0*A0+X1*A1+X2*A2-Y1*B0-Y2*B1
typedef const struct T_BQStageKoef
{
	float A0;
	float A1;
	float A2;
	float B0;
	float B1;
} TBQStageKoef;

typedef struct T_BQStageVar
{
	float X0;
	float X1;
	float X2;
	float Y0;
	float Y1;
	float Y2;
} TBQStageVar;

//Stages Koefficients: A0,A1,A2,B0,B1
//Max Number of Stages is 5
#define FiltStage0	{1.0, 0.0, -1.0, 1.831715, -.9825727}
#define FiltStage1	{1.0, 0.0, -1.0, 1.833676, -.9910886}
#define FiltStage2	{1.0, 0.0, -1.0, 1.845574, -.9914089}
#define	BQH	6.673715E-7

#define BQNum	sizeof(FiltKoef)/sizeof(float)/5

//Function Declaration
void BiquadShift(float* BQSet, float* Sample);
float BiquadCompute(TBQStageKoef* Koef, TBQStageVar* Var);
sint IIRFilter(uint Sample);
void BiquadInit(void);

#endif
