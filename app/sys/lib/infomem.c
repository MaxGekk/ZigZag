#include    <_zzSys.h>
#include    <syszig.h>
#include    <infomem.h>
#include    <msp430flash.h>

/*! @file   infomem.c
 *  @brief  Реализация доступа к сегментам infomem
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 1
 */   

#define     SEGA_ADDR   0x1080
#define     SEGA_SIZE   128

extern uint8_t _begin_module_infomem;

result_t umemcopy(void *const dst, const void *const src, const uint8_t len)
{
    if (((uint16_t)dst) - SEGA_ADDR < SEGA_SIZE)
    {
        return mem2flash(dst, src, len);
    }
    else
    {
        memcopy(dst, src, len);
        return ENOERR;
    }
}

/* запись в прикладной сегмент infomem 
 * pre: dst находится внутри сегмента
 */
result_t mem2flash(void *const dst, const void *const src, const uint8_t len)
{
    // TODO: переписать без использования дополнительной памяти
    uint8_t buf[SEGA_SIZE];
    infomem_read(buf);
    memcopy(buf + ((uint8_t*)dst - (uint8_t*)&_begin_module_infomem), src, len );
    infomem_write(buf);
    return ENOERR;
}

/* Вычитывание содержимого сегмента A */
result_t    infomem_read( void  *const ptr )
{
    memcopy( ptr, (void *)SEGA_ADDR, SEGA_SIZE );
    return ENOERR;
}

/* Запись в сегмент A
 * Функция будет корректно работать только если она запущена из flash-памяти */
result_t    infomem_write( const void   *const ptr )
{
    uint16_t *sega_ptr = (uint16_t *)(SEGA_ADDR);
    uint16_t *sega_eptr = (uint16_t *)(SEGA_ADDR+SEGA_SIZE);
    const uint16_t *data_ptr = ptr;

    if( !ptr ) return EINVAL;

    if( ISZIGLOAD ) __critical_enter();
    else asm("dint");

    /* Очищаем сегмент A */
    FCTL2 = FWKEY | FSSEL_MCLK | 25;    /* Источник тактирования MCLK = 8MHz. Делитель 25. ~300KHz */
    FCTL3 = FWKEY;                  /* Сбрасываем бит LOCK и таким образом разрешаем очистку/запись */
    FCTL1 = FWKEY | ERASE;          /* Говорим, что будем стирать один сегмент */
    *sega_ptr = 0xFFFF;            /* Инициируем стирание */
    FCTL1 = FWKEY;
    FCTL3 = FWKEY | LOCK;           /* Залочиваем flash обратно */
    /* Записываем сегмент A пословно */
    while( sega_ptr < sega_eptr ) {
        FCTL2 = FWKEY | FSSEL_MCLK | 25;
        FCTL3 = FWKEY;
        FCTL1 = FWKEY | WRT;        /* Говорим, что будем записывать */
        *sega_ptr++ = *data_ptr++;
        FCTL1 = FWKEY;
      	FCTL3 = FWKEY | LOCK;
    }
    if( ISZIGLOAD ) __critical_exit();
    else asm("eint");
   
    return ENOERR; 
}

