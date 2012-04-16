/*! @file   mbdl.c
 *  @brief  Общий код канального уровня ModBus
 *  @author Max Gekk
 *  @date   январь 2008
 *  @version 1
 *  */

#include    <_zigzag.h>

/*  Функция подсчета СRC16
 *  data - указатель на данные
 *  size - размер данных. Значение аргумента должно быть не более 65535 байт
 *  Функция возвращает подсчитанное значение CRC16   
 * */
#define POLINOME_CRC16  0xA001
uint16_t mb_crc16(uint8_t *data, uint16_t size )
{ 
    uint16_t i, j, crc = 0xFFFF;
    for( i=0; i<size; i++ ) { 
        crc=crc^data[i];
        for( j=0; j<8; j++ ) { 
            if ( (crc&0x0001) != 0x0000 ) 
                crc = (crc>>1)^POLINOME_CRC16;
            else crc>>=1;
        }
    }
    return (crc);
}

