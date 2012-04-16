//  vim:encoding=utf-8
#ifndef     _MODBUS_H
#define     _MODBUS_H
/*! @file   modbus.h
 *  @brief  Интерфейс работы с шиной ModBus.
 *  @author Max Gekk
 *  @date   декабрь 2007
 *  @version 2
 *  @defgroup MODBUS Интерфейс к шине ModBus
 *  @{ */

#include    <_zigzag.h>

/*! @defgroup MODBUS_DATALINK Интерфейс канального уровня ModBus
 *  @ingroup  MODBUS
 *  @{ */

/* Максимальный размер буфера под фреймы modbus */
#define     MB_BUF_SIZE    101
/* Функциональный код запроса мастером имеющихся на слейве данных */
#define     MB_FCODE_REQUEST    100     
/* Функциональный код ответа на запрос мастером данных */
#define     MB_FCODE_ANSWER     101     

/*! @fn uint16_t mb_crc16(uint8_t *data, uint16_t size );
 *  @brief  Функция подсчета СRC16
 *  @param  data - указатель на данные
 *  @param  size - размер данных. Значение аргумента должно быть не более 65535 байт
 *  @return Функция возвращает подсчитанное значение CRC16   
 * */
uint16_t mb_crc16(uint8_t *data, uint16_t size );

/*! @defgroup MODBUS_DL_MASTER Интерфейс мастера шины ModBus 
 *  @ingroup  MODBUS_DATALINK
 *  @{ */

/*! @fn result_t    mb_master_request( uint8_t slave_addr, uint8_t req_fcode, void    *req_dptr,  uint8_t req_dsize, uint8_t *ans_fcode,  void   *ans_dptr,   uint8_t *ans_dsize );
 *  @brief  Запрос к подчинённому устройству на шине ModBus.
 *      После завершения выполнения запроса генерируется широковещательное событие EV_MBREQ_DONE.
 *  В аргументе unidata обработчика этого события содержится результат запроса типа result_t.
 *  @param  slave_addr - адрес подчинённого устройства
 *  @param  req_fcode - функциональный код запроса
 *  @param  req_dptr - указатель на данные запроса
 *  @param  req_dsize - размер даннных запроса
 *  @param  ans_fcode - указатель на область памяти, в которую будет скопирован 
 *  функциональный код ответа. Если указатель равен нулю, то ответ ожидаться не будет.
 *  @param  ans_dptr - указатель на область памяти, в которую будут скопированы данные
 *  ответа подчинённого узла. Если указатель равен нулю, то ответ ожидаться не будет.
 *  @param  ans_dsize - указатель на максимально допустимый размер данных ответа.
 *  После ответа по указателю записывается реальный размер принятых данных.
 *  Если указатель равен нулю, то ответ ожидаться не будет.
 *  @return При успешном инициировании запроса возвращается ENOERR.
 *  Если драйвер modbus уже занят обработкой запроса, то возвращается EBUSY.
 * */
result_t    mb_master_request( uint8_t slave_addr, 
        uint8_t req_fcode, void    *req_dptr,  uint8_t req_dsize,
        uint8_t *ans_fcode,  void   *ans_dptr,   uint8_t *ans_dsize );

/*! @}  */
/*  @} */

/*! @defgroup MODBUS_PHY_TX Интерфейс передачи фреймов 
 *  @ingroup  MODBUS
 *  @{ */

/*! @fn uint8_t*    mb_get_txbuf();
 *  @brief  Получить указатель на буфер отправки
 *  @return Возвращается 0 в случае ошибки. Если в результате выполнения
 *  функии ошибки не произошло, то возвращается ненулевой указатель на буфер
 *  отправки.
 * */
uint8_t*    mb_get_txbuf();

/*! @fn result_t    mb_tx( uint8_t  size );
 *  @brief  Передать указанное число байт из буфера отправки
 *  @param  size - размер данных ( в байтах ), которые нужно отправить 
 *  по шине ModBus из буфера отправки ( с начала буфера ).
 *  @return В случае успеха возвращается ENOERR, иначе код ошибки < 0.
 * */
result_t    mb_tx( size_t  size );

/*! @fn void    mb_tx_done( result_t    result );
 *  @brief  Уведомление об окончании процедуры отправки
 *  @param  result - Результат выполнения отправки.
 *      - ENOERR - ошибок нет
 *      - EABORT - процедура отправки была прервана
 * */
void    mb_tx_done( result_t    result );

/*  @}  */

/*! @defgroup MODBUS_PHY_RX Интерфейс приёма фреймов 
 *  @ingroup  MODBUS
 *  @{ */

/*! @fn void    mb_rx_notify( uint8_t   *buf, uint8_t   size );
 *  @brief  Уведомление о приёме фрейма по шине ModBus
 *      Дальнейший приём данных не производится до тех пор, пока не будет
 *  вызвана функция mb_rx_enable.
 *  @param  buf - указатель на буфер с принятым фреймом
 *  @param  size - размер принятого фрейма. Если аргумент равен 0, то это
 *  означает, что истёк интервал ожидания данных.
 * */
void    mb_rx_notify( uint8_t   *buf, uint8_t   size );

/*! @fn void    mb_rx_enable( uint16_t  wait_duration );
 *  @brief  Разрешить дальнейший приём данных
 *  @param  wait_duration - время ожидания данных в миллисекундах.
 *  Если аргумент равен нулю, то никаких ограничений на время ожидания данных не устанавливается.
 *  После инициализации приём разрешён, при этом wait_duration = 0.
 * */
void    mb_rx_enable( uint16_t  wait_duration );

/*  @} */

/*! @defgroup MODBUS_ASCII Интерфейс приёма фреймов 
 *  @ingroup  MODBUS
 *  @{ */

/*! @fn int mbascii_tx( unsigned char mb_addr, unsigned char func_code, int size, unsigned char  *data );
 *  @brief  Отправка данных по протоколу ModBus ASCII
 *  @param  mb_addr - адрес источника
 *  @param  func_code - функциональный код
 *  @param  size - размер отправляемых данных
 *  @param  data - указатель на данные. 
 *  @return В случае успеха возвращается 0, иначе значение < 0.
 * */
int mbascii_tx( unsigned char mb_addr, unsigned char func_code,
        int size, unsigned char  *data );

/*! @fn void mbascii_rx_notify( unsigned char mb_addr, unsigned char func_code, int size, void  *data );
 *  @brief  Уведомление о приёме данных
 *  @param  mb_addr - адрес назначения
 *  @param  func_code - функциональный код
 *  @param  size - размер принятых данных
 *  @param  data - указатель на принятые данные. После выхода из функции указатель на данные
 *  будет некорректным.
 * */
void mbascii_rx_notify( unsigned char mb_addr,
        unsigned char func_code,
        int size, void  *data ); 

/*  @} */

/*  @} */
#endif  /*  _MODBUS_H   */
