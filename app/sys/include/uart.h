#ifndef     _ZZ_UART_H
#define     _ZZ_UART_H
/*! @file   uart.h
 *  @brief  Интерфейс работы с uart
 *  @author Max Gekk
 *  @date   март 2007
 *  @version 1
 */   

/*! @fn void uart_start();
 *  @brief  Запуск UART
 * */
void uart_start();

/*! @fn void uart_stop();
 *  @brief  Остановка UART-a
 * */
void uart_stop();

/*! @fn void uart_tx(const uint8_t data);
 *  @brief  Отправка байта данных по UART
 * */
void uart_tx(const uint8_t data);

/*! @fn void uart_tx_done();
 *  @brief  Уведомление о завершении отправки байта данных
 * */
void uart_tx_done();

/*! @fn void uart_rx_done( uint8_t data );
 *  @brief  Уведомление о приёме одного байта по UART-у
 * */
void uart_rx_done( uint8_t data );

#endif      /* _ZZ_UART_H */

