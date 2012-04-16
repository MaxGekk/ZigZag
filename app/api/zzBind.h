#ifndef _ZZ_BIND_H
#define _ZZ_BIND_H

/*! @file   zzBind.h
 *  @brief  Связывание объекта с портом
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 5
 */   

#include    <_zzBind.h>

#if !defined( OBJ )
#error "OBJ not defined"
#endif

#if !defined( PORT )
#error "PORT not defined"
#endif

OBJ_INFO( OBJ, PORT );

#define	OBJ_OFFSET	OBJ_OFFSET0( OBJ )

#endif  /*  _ZZ_BIND_H  */

