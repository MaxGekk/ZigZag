#ifndef _ZZ_INIT_H
#define _ZZ_INIT_H

/*! @file   _zzInit.h
 *  @brief  Инициализация подсистем прикладного уровня
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 

 *  @defgroup init Инициализация @{ 
 *      Интерфейс предоставляет макросы для регистрации функций инициализации подсистем
 * */

/*! @typedef void (* init_func_t )()
 *  @brief Тип функции - инициализатора модуля.
 * */
typedef void (*init_func_t) ();

extern init_func_t __init_start;    /*!< Указатель на первую функцию-инициализатор */
extern init_func_t __init_end;  /*!< Указатель на последнюю функцию-инициализатор */

/*! @def PRIORITY_INIT(fn,priority)
 *  @brief Регистрация функции в качестве инициализатора модуля. Функция вызывается при запуске системы.
 *  @param fn - регистрируемая функция
 *  @param priority - приоритет вызова функции @a fn. Чем меньше лексикографическое значение @a priority, тем
 *                    раньше будет вызвана функция @a fn.
 * */
#define PRIORITY_INIT(fn,priority) init_func_t __init_##fn __attribute__((unused,__section__ ("." #priority ".initappsys"))) = fn;

/*! @def INIT(fn)
 *  @brief Регистрация функции в качестве инициализатора. Функция вызывается при запуске системы.
 *  @param fn - регистрируемая функция
 * */
#define INIT(fn) PRIORITY_INIT(fn, A);

/*! @} */
#endif                          /* _ZZ_INIT_H */

