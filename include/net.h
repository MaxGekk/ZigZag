#ifndef     _NET_H
#define     _NET_H

/*! @file   net.h
 *  @brief  Функции сетевого взаимодействия
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 
 */

/*! @defgroup network_op Вход и выход из сети @{ */

/*! @fn void    __net_enter( uint16_t pan_id );
 *  @brief  Извещение о входе в сеть
 *  @param pan_id - идентификатор сети
 * */
void    __net_enter( uint16_t pan_id );

/*! @fn void    __net_exit();
 *  @brief  Извещение о выходе из сети
 * */
void    __net_exit();

/*! @}  */

/*! @defgroup network Приём и передача сетевых пакетов @{ */

/*! @fn int16_t __net_send( const uint16_t dst_addr, const uint16_t data_size, const uint8_t *const data, uint8_t handle );
 *  @brief  Отправка сетевого пакета
 *  @param dst_addr - сетевой адрес назначения
 *  @param data_size - размер данных
 *  @param data - указатель на область памяти с данными
 *  @param handle - описатель пакета
 *  @return Возвращается 0 в случае успешного начала процедуры отправки, иначе -1.
 * */
int16_t __net_send( const uint16_t dst_addr, const uint16_t data_size,
        const uint8_t *const data, uint8_t handle );

/*! @fn void    __net_send_done( uint8_t handle, int8_t status );
 *  @brief  Оповещение об окончании процедуры отправки
 *  @param handle - описатель пакета
 *  @param status - статус завершения процедуры отправки. 0 - если пакет был успешно отправлен,
 *  иначе отрицательное значени.
 * */
void    __net_send_done( uint8_t handle, int8_t status );

/*! @fn void    __net_recv( uint16_t src_addr, uint64_t src_ext_addr, uint16_t data_size, uint8_t data, uint8_t lqi );
 *  @brief  Оповещение о приёме пакета
 *  @param src_addr - адрес источника
 *  @param src_ext_addr - длинный адрес источника
 *  @param data_size - размер данных
 *  @param data - указатель на область памяти с принятыми данными.
 *  @param lqi - качество связи при приёме данного пакета
 * */
void    __net_recv( uint16_t src_addr, uint64_t src_ext_addr, uint16_t data_size, uint8_t *data, uint8_t lqi );

/*! @fn uint16_t    __net_addr();
 *  @brief  Функция возвращает сетевой адрес узла
 * */
uint16_t    __net_addr();

/*  @}  */

/*! @defgroup network_child Присоединение и отсоединение дочерних узлов @{ */

/*! @fn void    __net_child_join( uint16_t short_addr, uint64_t ext_addr )
 *  @brief  Извещение о присоединении дочернего узла 
 *  @param short_addr - короткий ( сетевой ) адрес присоединившегося узла
 *  @param ext_addr - длинный адрес присоединившегося дочернего узла
 *  @param lqi - качество связи с присоединённым узлом
 * */
void    __net_child_join( uint16_t short_addr, uint64_t ext_addr, uint8_t lqi );

/*! @fn void    __net_child_leave( uint64_t ext_addr )
 *  @brief  Извещение об отсоединении дочернего узла
 *  @param  ext_addr - длинный адрес отсоединившегося узла
 * */
void    __net_child_leave( uint64_t ext_addr );

/*! @}  */

/*! @defgroup network_serv Сервисные функции
 *  @{ */

/*! @fn int16_t __net_sleep( uint32_t duration );
 *  @brief  Извещение о засыпании стека
 *  @param duration - продолжительность сна в миллисекундах
 *  @return 0 - в случае успеха, -1 - ошибка.
 * */
int16_t __net_sleep( uint32_t duration );

/*! @}  */

#endif  /*  _NET_H  */

