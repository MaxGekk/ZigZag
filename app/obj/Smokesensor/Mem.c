#include  <io.h>
#include "typedefs.h"
#include "Mem.h"

void	IEEWR(void* Src, void* Dst, uint Len)
{
//__disable_interrupt();
	FCTL1 = FWKEY + ERASE;                    		// Set Erase bit
	FCTL2 = FWKEY + FSSEL_1 + FN0 + FN1 + FN2;    // MCLK/7 for Flash Timing Generator
  FCTL3 = FWKEY;                            		// Clear Lock bit
  *((uchar*)Dst) = 0;                   				// Dummy write to erase Flash segment
	while(FCTL3 & BUSY);
  FCTL1 = FWKEY + WRT;                      		// Set WRT bit for write operation
  while(Len--)
  {
    *((uchar*)Dst) = *((uchar*)Src);						// Write value to flash
		*((uchar*)&Dst)+=1;
		*((uchar*)&Src)+=1;
		while(FCTL3 & BUSY);
  }
  FCTL1 = FWKEY;                            		// Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     		// Set LOCK bit
//__enable_interrupt();
}

