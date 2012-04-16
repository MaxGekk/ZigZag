#ifndef     __ZZ_MACRO_H
#define     __ZZ_MACRO_H

#define REG_CALLBACK( fn ) _REG_CALLBACK(fn, OBJ )
#define _REG_CALLBACK( fn, OBJ ) __REG_CALLBACK( fn, OBJ )
#define __REG_CALLBACK( fn, OBJ ) static hword_t _##OBJ##_##fn __attribute__ ((unused,__section__ ( #OBJ "." #fn ))) = (hword_t)fn;

#define     __callback  static __attribute__((used)) 
#define     __syscall  static inline

#endif  /*  __ZZ_MACRO_H    */
