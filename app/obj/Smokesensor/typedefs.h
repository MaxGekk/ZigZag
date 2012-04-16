#ifndef	typedefs_h
#define	typedefs_h
typedef unsigned char uchar;
typedef signed char schar;
typedef unsigned int uint;
typedef signed int sint;
typedef unsigned long ulong;
typedef signed long slong;
#define DWORD unsigned long
#define WORD unsigned int
#define BOOL  unsigned char

typedef union T_Word
{
	uint	UVal;
	sint	SVal;
	struct
	{
		uchar Byte0;
		uchar Byte1;
	};
	struct
	{
		uchar L;
		uchar H;
	};
}word;

typedef union T_DWord
{
	ulong	UVal;
	slong	SVal;
	struct
	{
		uchar Byte0;
		uchar Byte1;
		uchar Byte2;
		uchar Byte3;
	};
	struct
	{
		uchar L0;
		uchar H0;
		uchar L1;
		uchar H1;
	};
} dword;

typedef struct _BITS
{
	uchar b0:1;
	uchar b1:1;
	uchar b2:1;
	uchar b3:1;
	uchar b4:1;
	uchar b5:1;
	uchar b6:1;
	uchar b7:1;
}bits;

typedef struct _BITS8
{
	uchar b0:1;
	uchar b1:1;
	uchar b2:1;
	uchar b3:1;
	uchar b4:1;
	uchar b5:1;
	uchar b6:1;
	uchar b7:1;
}bits8;

typedef struct _BITS16
{
	uint  b0:1;
	uint  b1:1;
	uint  b2:1;
	uint  b3:1;
	uint  b4:1;
	uint  b5:1;
	uint  b6:1;
	uint  b7:1;
	uint  b8:1;
	uint  b9:1;
	uint  bA:1;
	uint  bB:1;
	uint  bC:1;
	uint  bD:1;
	uint  bE:1;
	uint  bF:1;
}bits16;

//#define TRUE  1
//#define FALSE 0

#endif
