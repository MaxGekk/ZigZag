//  vim:encoding=utf-8
/*! @file   mbphy_ascii.c
 *  @brief  Реализация ModBus ASCII
 *  @author Max Gekk
 *  @date   март 2007
 *  @version 1
 */   

#include    <_zigzag.h>
#include    <modbus.h>
#include    <syszig.h>
#include    <uart.h>

//#define     LED_PORT     2       /* номер порта светодиодов */
//#define     RED_LED     0x80    /* Ножка красного светодиода */

/* Общее число буферов на приём и передачу */
#define     TOTAL_BUFFERS   4
/* Размер буфера */
#define     BUFFER_SIZE     MB_BUF_SIZE
/* Размер дополнительных полей в modbus-фрейме: адрес, функ. код и контрольная сумма */
#define     MODBUS_DL_FIELD_SIZE    3

/* Состояния буфера */
/* Буфер свободен */
#define     BUFFER_IS_FREE  0
/* Буфер выделен */
#define     IDLE            1
/* Производится приём данных */
#define     RECEPTION       2
/* Приём конца фрейма */
#define     WAIT_END_FRAME  3
/* Обработка принятого фрейма  */
#define     PROCESS_FRAME   4
/* В буфере сформирован фрейм для отправки */
#define     TX_READY        5
/* Отправка данных из буфера */
#define     TX_DATA         6
/* Передача конца фрейма */
#define     TX_END_FRAME    7
/* Передача символа возврата коретки */
#define     TX_CR           8

/* Тип буфера */
typedef struct {
    unsigned char   state;  /* Состояние буфера */
    unsigned char   size;   /* Текущий размер данных в буфере */
    unsigned char   data[ BUFFER_SIZE ];    /* Данные буфера */
} buffer_t;

static  buffer_t    buffer[ TOTAL_BUFFERS ];
/* Указатель на область памяти за массивом буферов */
static  const   buffer_t *const inv_buf = &buffer[ TOTAL_BUFFERS ];
/* Указатель на текущий приёмный буфер */
static  buffer_t    *cur_ibuf;
/* Указатель на текущий отправляемый буфер */
static  buffer_t    *cur_obuf;

/* Функция выделяет буфер */
static  buffer_t*    alloc_buf()
{
    buffer_t    *free = buffer;
    while( free != inv_buf ) {
        if( free->state == BUFFER_IS_FREE ) {
            free->state = IDLE;
            return free;
        }
        free++;
    }
    return (buffer_t *)0;
}

/* Продолжение обработки принятого фрейма в задаче */
static  void    continue_process_input()
{
    buffer_t    *ibuf;
    /* Обрабатываем все принятые фреймы */
    for( ibuf=buffer; ibuf != inv_buf; ibuf++ ) {
        if( ibuf->state == PROCESS_FRAME ) {
            unsigned char size = ibuf->size >> 1;
            unsigned char j,LRC = 0;
            /* Подсчёт контрольной суммы */
            for( j=0; j<(size-1); j++ )
                LRC += ibuf->data[j];
            LRC = (unsigned char)(-(signed char)LRC);
            if( (MODBUS_DL_FIELD_SIZE<=size)&&(LRC == ibuf->data[size-1]) )
                mbascii_rx_notify( ibuf->data[0], ibuf->data[1], 
                        size-MODBUS_DL_FIELD_SIZE, &(ibuf->data[2]) );
            if( (buffer_t *)0 == cur_ibuf ) {
                ibuf->state = IDLE;
                cur_ibuf = ibuf;
            } else 
                ibuf->state = BUFFER_IS_FREE;
        }
    }
}

/* Обработка принятого байта */
static inline void    process_input( unsigned char c )
{
    if( (buffer_t *)0 == cur_ibuf )
        return;

    switch(c) {
        case ':':
            cur_ibuf->state = RECEPTION;
            cur_ibuf->size = 0;
        return;
        case '\r':
            if( cur_ibuf->state == RECEPTION )
                cur_ibuf->state = WAIT_END_FRAME;
        return;
        case '\n':
            if( cur_ibuf->state != WAIT_END_FRAME )
                return;
            if( IS_OK(__post_task(continue_process_input)) ) {
                cur_ibuf->state = PROCESS_FRAME;
                cur_ibuf = alloc_buf();
            }
        return;
        case '0'...'9': c = c - '0'; break;
        case 'a'...'f': c = (c - 'a') + 0xa; break;
        case 'A'...'F': c = (c - 'A') + 0xA; break;                  
        default: 
        return;                
    }
    if( cur_ibuf->state == RECEPTION ) {
        if( cur_ibuf->size & 0x01 )
            cur_ibuf->data[ cur_ibuf->size >> 1 ] |= c & 0x0F;
        else
            cur_ibuf->data[ cur_ibuf->size >> 1 ] = c << 4;
        cur_ibuf->size++;
    }
}

void uart_rx_done( uint8_t data )
{
    process_input( data );
}

static void mbascii_tx_char();

/* Поиск следующего буфера, готового к отправке и инициирование его отправки */
static void mbascii_tx_next()
{
    if( (buffer_t *)0 == cur_obuf ) {
        buffer_t    *next;
        for( next=buffer; next!=inv_buf; next++ ) 
            if( next->state == TX_READY ) {
                cur_obuf = next;
                mbascii_tx_char();
                return;
            }
    }
}

/* Преобразование старших 4-х бит в hex-символ */
static inline unsigned char hi_to_ascii( unsigned char d )
{
    d >>= 4;
    if( d <= 9 ) return '0'+d;
    else return 'A'+(d-0xA);
}

/* Преобразование младших 4-х бит в hex-символ */
static inline unsigned char lo_to_ascii( unsigned char d )
{
    d = d & 0xF;
    if( d <= 9 ) return '0'+d;
    else return 'A'+(d-0xA);
}

/* Отправка очередного байта из буфера. Перед отправкой производится его 
 * кодирование в hex-символ */
static void mbascii_tx_char()
{
    static  unsigned int  char_pos = 0;

    if( (buffer_t *)0 == cur_obuf )
        return;
    switch( cur_obuf->state ) {
        case TX_READY:
            uart_tx(':');
            cur_obuf->state = TX_DATA;
            char_pos = 0;
        return;
        case TX_DATA:
            if( char_pos & 0x01 )
                uart_tx( lo_to_ascii( cur_obuf->data[char_pos>>1] ) );
            else
                uart_tx( hi_to_ascii( cur_obuf->data[char_pos>>1] ) );
            ++char_pos;
            if( cur_obuf->size <= (char_pos>>1) )
                cur_obuf->state = TX_END_FRAME;
        return;
        case TX_END_FRAME:
            uart_tx('\r');
            cur_obuf->state = TX_CR;
        return;
        case TX_CR:
            uart_tx('\n');
            cur_obuf->state = BUFFER_IS_FREE;
            cur_obuf = (buffer_t *)0;
            __post_task( mbascii_tx_next );
        return;    
    }
}

void    uart_tx_done()
{
    mbascii_tx_char();
}

/* Передача данных по ModBus ASCII. Функция выделяет буфер, формирует фрейм и 
 * подсчитывает контрольную сумму */
int mbascii_tx( unsigned char mb_addr, unsigned char func_code,
        int size, unsigned char  *data )
{
    unsigned char *dptr, *lrc_ptr;
    buffer_t    *obuf;
    
    if( BUFFER_SIZE < (MODBUS_DL_FIELD_SIZE+size) )
        return -1;
    obuf = alloc_buf();
    if( (buffer_t *)0 == obuf )
        return -1;
    dptr = obuf->data;
    lrc_ptr = dptr+sizeof(mb_addr)+sizeof(func_code)+size;
    *lrc_ptr = 0;
    *dptr++ = mb_addr; *lrc_ptr += mb_addr;
    *dptr++ = func_code; *lrc_ptr += func_code;
    while( dptr < lrc_ptr ) {
        *lrc_ptr += *data;
        *dptr++ = *data++;
    }
    *lrc_ptr = (unsigned char)(-(signed char)*lrc_ptr);
    obuf->size = MODBUS_DL_FIELD_SIZE+size;
    obuf->state = TX_READY;
    mbascii_tx_next();
    return 0;
}

void    init_modbus_ascii()
{
//    port_attr_t   port_attr;
    int i;
    for( i=0; i<TOTAL_BUFFERS; i++ )
        buffer[i].state = BUFFER_IS_FREE;
    cur_ibuf = alloc_buf();
    cur_obuf = (buffer_t *)0;
    uart_start();
/*
    PIN_SET( port_attr.dir, RED_LED, PIN_HI );
    PIN_CLEAR( port_attr.sel, RED_LED);       
    PIN_CLEAR( port_attr.ie, RED_LED);        
    port_set_attr( LED_PORT, RED_LED, &port_attr );
    port_write( LED_PORT, RED_LED, PIN_HI );
*/
}
INIT( init_modbus_ascii );

