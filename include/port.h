#ifndef     _PORT_H
#define     _PORT_H

/*! @file   port.h
 *  @brief  Доступ к портам ввода/вывода
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 
 */

/*! @defgroup port Интерфейс взаимодействия с портами ввода/вывода @{ */

/* Операции с портами */
enum {
    OP_WRITE = 0,       /*!< Запись в порт */
    OP_READ,            /*!< Чтение из порта */
    OP_SET_ATTR,        /*!< Установка атрибутов порта */
    OP_GET_ATTR,        /*!< Запрос атрибутов порта */
    OP_GET_IFLAG,       /*!< Запрос флага прерывания */
    OP_RESET_IFLAG      /*!< Сброс флага прерывания */
};

/*! @fn int16_t __port_perm( const uint8_t port_num, const uint8_t mask, const uint8_t op );
 *  @brief  Проверка правомочности операции с портом
 *  @param  port_num - номер порта
 *  @param  mask - маска тех входов/выходов порта, правомочность операции с которыми проверяется
 *  @param  op - операция с портом
 *  @return Если операция правомочна возвращается 0, иначе отрицательное значение.
 * */
int16_t __port_perm( const uint8_t port_num, const uint8_t mask, const uint8_t op );

/*! @} */
#endif  /*  _PORT_H */

