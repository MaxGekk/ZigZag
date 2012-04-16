#ifndef     _ZZ_TYPES_H
#define     _ZZ_TYPES_H

/*! @file   zzTypes.h
 *  @brief  Объявление типов данных системы ZigZag
 *  @author Max Gekk
 *  @date   июнь 2007
 *  @version 5
 */   

/*! @defgroup types Типы данных системы ZigZag
 *  @ingroup ZIGZAG
 *  @{
 * */

#include    <inttypes.h>

#define NULL ((void*)0)

typedef uint16_t    hword_t;      /*!<    Аппаратное слово  */

/*! Логический тип данных */
typedef enum {
    FALSE   =   0,
    TRUE    =   1
} bool_t;

/*! @typedef result_t
 *  @brief  Результат выполнения вызова функции
 * */
typedef enum {
    MAX_RESULT  =   32768,  /*!< Максимальное значение типа res_t */   
    ENOERR  =   0,      /*!< Нет ошибки */ 
    ENOSYS  =   -1,     /*!< В текущей версии функция не поддерживается */
    EINVAL  =   -2,     /*!< Некорректное значение аргумента функции */   
    EACCESS =   -3,     /*!< Отказ в доступе */
    ENOMEM  =   -4,     /*!< Недостаточно памяти */   
    EBUSY   =   -5,     /*!< Ресурс занят */
    EPERM   =   -6,     /*!< Операция не разрешена */
    ENOTFOUND   =   -7, /*!< Запрошенные данные не найдены */
    EABORT  =   -8      /*!< Выполнение операции было прервано */
} result_t;

/*! @def IS_ERROR(res)
 *  @brief Является ли возвращённое значение ошибкой или нет.
 *  @param res результат выполнения функции
 *  @return 
 *      - 0<  произошла ошибка
 *      - 0   ошибки нет
 * */
#define     IS_ERROR( res )   ( (res) < 0 )

/*! @def IS_OK(res)
 *  @brief Является ли резльтат успешным или нет.
 *  @param res результат выполнения функции
 *  @return
 *      - 0<  результат был успешным
 *      - 0   ошибочный результат
 * */
#define     IS_OK( res )    ( 0 <= (res) )

typedef uint32_t    unidata_t;  /*!< Тип аргумента или переменной, содержащей дополнительные данные. Например о событии */

/*! Тип приоритета */
typedef enum {
    PRIORITY_LOW    =   0,      /*!<    Наименьший приоритет */
    PRIORITY_HIGH   =   127     /*!<    Наивысший приоритет */
} priority_t;     

/*! Информация о таймере */
struct timerinfo {
    uint8_t     is_set;     /*!< установлен ли таймер. 0 - если не установлен, > 0 - если установлен */
    uint32_t    tpoint;     /*!< время, на которое установлен таймер */
};

/*! Тип событий */
typedef enum {
    EV_USER_FIRST   = 0x00,
    EV_USER_LAST    = 0xdf,
    EV_ADC          = 0xf7, /*!< Событие от АЦП, в доп. аргументе содержится номер канала */
    EV_MBREQ_DONE   = 0xf8, /*!< Обработка запроса к шине ModBus завершена */
    EV_AWRITTEN     = 0xf9, /*!< Записано новое значение атрибута */
    EV_WUNLOCK      = 0xfa, /*!< Завершён доступ на запись к области памяти хранилища. */
    EV_RUNLOCK      = 0xfb, /*!< Завершён доступ на чтение к области памяти хранилища.*/  
    EV_SFREE        = 0xfc, /*!< Освобождена область памяти хранилища. */  
    EV_SLEEP        = 0xfd, /*!< Узел переходит в состояние сна. */
    EV_LEAVE        = 0xfe, /*!< Узел покинул сеть */
    EV_JOIN         = 0xff  /*!< Узел присоединился к сети */
} event_type_t;

/*! @typedef irq_t
 *  @brief  Номера прерываний
 * */
typedef enum {
    IRQ_DAC     =   0,
    IRQ_DMA     =   0,
    IRQ_PORT2   =   2,
    IRQ_PORT1   =   8,
    IRQ_ADC12   =   14,
    IRQ_COMPARATORA =   22
} irq_t;

typedef uint16_t    size_t;     /*!< Тип размера */

typedef uint16_t    net_addr_t; /*!< Тип сетевого адреса узла */
typedef uint64_t	ieee_addr_t; /*!< Длинный IEEE 802.15.4 адрес узла*/
typedef uint8_t     app_port_t; /*!< Тип номера прикладного порта  */

typedef	uint32_t    msg_t;

/*! Информация о сообщении */
struct  msginfo {
    net_addr_t      dst_addr;   /*!<  Адрес узла-назначения */
    app_port_t      dst_port;   /*!<  Порт объекта-назначения */
    net_addr_t      src_addr;   /*!<  Адрес узла-источника */
	ieee_addr_t		src_ext_addr;	/*!<  Длинный адрес узла-источника */
    app_port_t      src_port;   /*!<  Порт объекта-источника */
    uint8_t         msg_type;   /*!<  Тип сообщения */
    size_t          body_size;  /*!<  Размер тела сообщения */
    void            *body_ptr;  /*!<  Указатель на тело сообщения */
};

/*! @typedef    status_t
 *  @brief  Статус завершения какой-либо операции
 * */
typedef enum {
    STATUS_SUCCESS,     /*!<    Операция завершена успешно */
    STATUS_TIMEOUT,     /*!<    Истёк таймаут ожидания выполнения операции */
    STATUS_MAX_ATTEMPTS /*!<    Превышено максимальное число попыток выполнения операции */
} status_t;

/*! @typedef    bind_t
 *  @brief  Тип связки номера объекта и номера порта
 * */
typedef struct {
    uint16_t    obj_num;    /*!< Номер объекта */
    uint16_t    port_num;   /*!< Номер порта, с которым связан порт */
} bind_t;

typedef     uint8_t port_t; /*!< Тип данных, разрядность которого соответсвует разрядности порта */

/* Макросы, задающие ножки */
#define     PIN_0   (1 << 0 )
#define     PIN_1   (1 << 1 )
#define     PIN_2   (1 << 2 )
#define     PIN_3   (1 << 3 )
#define     PIN_4   (1 << 4 )
#define     PIN_5   (1 << 5 )
#define     PIN_6   (1 << 6 )
#define     PIN_7   (1 << 7 )

#define     PIN_HI  0xff
#define     PIN_LO  0x00

/* Сброс битов, соответсвующих ножкам, в переменной типа port_t */
#define     PIN_CLEAR( port, mask ) (port) &= ~(mask)
/* Установка битов, соответсвующих ножкам, в переменной типа port_t */
#define     PIN_SET( port, mask, pins )  { PIN_CLEAR(port, mask); (port) |= (pins)&(mask); }
/* Проверка установлены ли биты в переменной типа port_t */
#define     PIN_IS_SET( port, mask )    (( (port) & (mask) ) == (mask) )
/* Проверка: сброшены ли биты в переменной типа port_t */
#define     PIN_IS_RESET( port, mask )  ( (port)&(mask) == 0 )

/*! Атрибуты порта ввода/вывода */
typedef struct {
    port_t  dir;    /* Направление: 0 - ввод, 1 - вывод */
    port_t  sel;    /* Выбор функции: 0 - ввод/вывод, 1 - переферийный модуль */
    port_t  ie;     /* Разрешение прерываний: 0 - запрещено, 1 - разрешено */
    port_t  ies;    /* Фронт прерывания: 0 - с низкого на высокий, 1 - с высокого на низкий */
} port_attr_t;

/*! @} */
#endif  /*  _ZZ_TYPES_H */

