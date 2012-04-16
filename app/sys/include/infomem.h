#ifndef     _ZZ_INFOMEM_H
#define     _ZZ_INFOMEM_H
/*! @file   infomem.h
 *  @brief  Доступ к сегментам информационной памяти (infomem)
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 1
 */   

#include    <zzTypes.h>

/*! @defgroup ZIG_INFOMEM Интерфейс доступа к информационной памяти
 *  @ingroup ZIGZAG
 *  @{
 *      Функции данного интерфейса позволяют получить доступ только к
 *  сегменту А, который расположен по адресам от 0x1080-0x1100.
 * */

/*  @fn result_t    infomem_read( void  *const ptr );
 *  @brief Вычитывание сегмента A в ОЗУ
 *  @param ptr - указатель на область памяти в ОЗУ, в которую будет скопировано
 *  содержимое сегмента A. По указателю ptr должна быть выделена память в размере 128 байт.
 *  @return В случае успеха возвращается ENOERR, иначе значение < 0.
 * */
result_t    infomem_read( void  *const ptr );

/*  @fn result_t    infomem_write( const void   *const ptr );
 *  @brief  Запись в сегмент А
 *  @param ptr - указатель на область памяти, содержимое которой будет записано в сегмент A.
 *  @return В случае успеха возвращается ENOERR, иначе значение < 0.
 * */
result_t    infomem_write( const void   *const ptr );


/*  @fn result_t mem2flash(void *const dst, const void *const src, const uint8_t len)
 *  @brief  копирование памяти в сегмент А. Корректность атрибутов не проверяется.
 *  @param dst - указатель на область в сегменте A, куда будет осуществляться запись.
 *  @param src - указатель на область памяти, содержимое которой будет записано в сегмент A.
 *  @param len - длина копируемой области памяти.
 *  @return В случае успеха возвращается ENOERR, иначе значение < 0.
 * */
result_t mem2flash(void *const dst, const void *const src, const uint8_t len);


result_t umemcopy(void *const dst, const void *const src, const uint8_t len);

/*! @} */

#endif      /* _ZZ_INFOMEM_H */

