#ifndef   _SYSZIG_H
#define   _SYSZIG_H

/*! @file   syszig.h
 *  @brief  Интерфейс между прикладной и системной частью ZigZag-a
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 
 */

#include    <zzConst.h>
#include    "irq.h"
#include    "time.h"
#include    "task.h"
#include    "net.h"
#include    "port.h"

typedef uint16_t    faddr_t;
#define EXISTS( func )  ( (faddr_t)func != ((faddr_t)-1) )

extern  uint16_t    __module_load;
#define ISMODLOAD   EXISTS( __module_load )

extern  uint16_t    __zigzag_load;
#define ISZIGLOAD   EXISTS( __zigzag_load )

#endif  /*  _SYSZIG_H */

