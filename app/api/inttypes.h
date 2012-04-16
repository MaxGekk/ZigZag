#ifndef __INTTYPES_H_
#define __INTTYPES_H_

/*! @file   inttypes.h
 *  @brief  Объявление целочисленных типов данных
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 1
 */   

#ifndef __int8_t_defined
#define __int8_t_defined

typedef unsigned char uint8_t;  /*!< Неотрицательное целое из диапазона 0..255 */
typedef unsigned short int uint16_t;    /*!< Неотрицательное целое из диапазона 0..65535 */
typedef unsigned long uint32_t; /*!< Неотрицательное целое из диапазона 0..4294967295 */
typedef unsigned long long uint64_t;    /*!< Неотрицательное целое из диапазона 0..18446744073709551615 */

typedef signed char int8_t;     /*!< Целое число из диапазона -127..127 */
typedef signed short int int16_t;   /*!< Целое число из диапазона -32767..32767 */
typedef signed long int32_t;    /*!< Целое число из диапазона -2147483647..2147483647 */
typedef signed long long int64_t;   /*!< Целое число из диапазона -9223372036854775807..9223372036854775807 */

#endif

#ifndef __intptr_t_defined
#define __intptr_t_defined
typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
#endif

#define _MSP430_SIZE_T_	uint16_t

#endif

