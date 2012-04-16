
//#define DBGLOG
#include<_zigzag.h>
#include<zzDebug.h>
#include<msp430.h>


#define THR_ACTIVE_LEVELS(attr_num) (attr_num + 1)
#define THR_TRIGGERED_LEVELS(attr_num) (attr_num + 2)

//#define THR_UPPER(thr_num, attr) (attr + 6 + thr_num*2)
//#define THR_LOWER(thr_num, attr) (attr + 7 + thr_num*2)

//#define THR_UPGOING_OFFSET(thr_num, attr) (attr + 6 + 16 + thr_num*2)
//#define THR_DOWNGOING_OFFSET(thr_num, attr) (attr + 7 + 16 + thr_num*2)


#define THR_ATTR_NUM(attr_num, thr_num) (attr_num + 6 + thr_num)


// режимы оповещения о пересечении порогов

#define THR_IS_MODE_NUM(mode) (mode & 1)
#define THR_IS_MODE_READING_MASK(mode) (mode & 2)
#define THR_IS_MODE_READING_OFFSET_MASK(mode) (mode & 4)

// определяет номер показания по номеру атрибута:
// 0x24 -> 0
// 0x4A -> 1
// 0x70 -> 2
// 0x96 -> 3
#define READING_NUM(attr_num) ((attr_num-0x24)/0x26)

#define ATTR_IS_READING(attr_num) (((attr_num) == 0x24 || (attr_num) == 0x4A || \
								    (attr_num) == 0x70 || (attr_num) == 0x96))

void report_attr_change(const uint16_t objoffset, common_attr_t *ca, uint8_t attr_num);
void check_thresholds(uint16_t objoffset, uint8_t attr_num);

// разветвитель вызовов к объектным isGT()
uint16_t _isGT(const uint16_t objoffset, uint8_t attr_num1, uint8_t attr_num2)
{
    const isGT_ft *gt = &_begin_isGT;
    gt += objoffset;
    return (*gt)(attr_num1, attr_num2);
}


#define ATTR_REPORT_THR_STATE          1 //всегда срабатывает независимо от данного бита
#define ATTR_REPORT_READING_ON_THR     2 // not implemented
#define ATTR_REPORT_READING_ON_CHANGE  4 // not implemented

// TODO перенести в другой модуль?
void _attr_on_change(uint16_t objoffset, common_attr_t *ca, uint8_t attr_num)
{
    uint16_t report_mode;

    _event_emit(objoffset, PRIORITY_LOW, EV_AWRITTEN, attr_num);

	if (! ATTR_IS_READING(attr_num))
	{
		// сообщаем о любом изменении атрибутов, которые не являются измерениями
		report_attr_change(objoffset, ca, attr_num);
		return;
	}

    _attr_get(objoffset, ATTR_THRESHOLD_REPORT_MODE, &report_mode);
    DBG1("report_mode: %x", report_mode);

	// вычисляем режим данного измерения
    report_mode = (report_mode & (0xF << (READING_NUM(attr_num)*4)) >> READING_NUM(attr_num)*4);
    if ((report_mode & 0x4))
	{
		// Сообщаем об изменении показания
       	report_attr_change(objoffset, ca, attr_num);
		return;
	}
	
	check_thresholds(objoffset, attr_num);
}

void check_thresholds(uint16_t objoffset, uint8_t attr_num)
{
    uint16_t active_levels;
    uint16_t old_triggered_levels;
	uint16_t new_triggered_levels;
    uint16_t pair_offset;

    // получаем включенные пределы
    _attr_get(objoffset, THR_ACTIVE_LEVELS(attr_num), &active_levels);
    
    // получаем флаги сработавших порогов (для них не надо генерировать сообщение повторно)
    _attr_get(objoffset, THR_TRIGGERED_LEVELS(attr_num), &old_triggered_levels);
	new_triggered_levels = old_triggered_levels;
    
    //DBG2(" active %x triggered %x", active_levels, triggered_levels);

    // пройдем по всем парам порогов (нижний, верхний)
    
    pair_offset = 0;

    for (;pair_offset < 16; pair_offset+=2)
    {
        uint8_t high_active = active_levels & (1 << pair_offset);
        uint8_t low_active  = active_levels & (2 << pair_offset);
        uint8_t high_reported = old_triggered_levels & (1 << pair_offset);
        uint8_t low_reported  = old_triggered_levels & (2 << pair_offset);

        //DBG1("pair %x", pair_offset);
        //DBG4("ha %x, la %x, hr %x, lr %x", high_active, low_active, high_reported, low_reported);

        // проверяем активность и наличие порога (если его нет, размер атрибута == 0)
        if (high_active && (_attr_size(objoffset, THR_ATTR_NUM(attr_num, pair_offset)) != 0))
        {
            if (_isGT(objoffset, attr_num, THR_ATTR_NUM(attr_num, pair_offset)))
            {   // выше верхнего порога
                low_reported = FALSE;
                high_reported = TRUE;
            }
            else
            {   // ниже верхнего
                if (! low_active)
                    high_reported = FALSE;
            }
        }
        // проверяем активность и наличие порога (если его нет, размер атрибута == 0)
        if (low_active && (_attr_size(objoffset, THR_ATTR_NUM(attr_num, pair_offset)) != 0))
        {
            if (! _isGT(objoffset, attr_num, THR_ATTR_NUM(attr_num, pair_offset + 1)))
            {   // ниже нижнего порога
                high_reported = FALSE;
                low_reported = TRUE;
            }
            else
            {   // выше нижнего
                if (! high_active)
                    low_reported = FALSE;
            }        
        }
        high_reported = high_reported?1:0;
        low_reported = low_reported?2:0;

        new_triggered_levels = (new_triggered_levels & (~(3<<pair_offset))) | ((high_reported | low_reported) <<pair_offset);
    }

	// если что-то изменилось, записываем новое значение в атрибут
    if (new_triggered_levels != old_triggered_levels)
	{
        _attr_set(objoffset, THR_TRIGGERED_LEVELS(attr_num), &new_triggered_levels);
		// TODO тут при правильном режиме генерации можно вызвать оповещение
		// об изменении порога-измерения
		// report_attr_change(...)
        DBG1("reported levels %x", new_triggered_levels);
	}

}


void report_attr_change(const uint16_t objoffset, common_attr_t *ca, uint8_t attr_num)
{
    #define _MAX_ATTR_SIZE 8 
    //uint8_t max_len = HEADER_SIZE + _attr_size(objoffset, attr_num);
    {
        //uint8_t buf[max_len]; // глюк компилятора :(
		if (_attr_size(objoffset, attr_num) > _MAX_ATTR_SIZE)
			return;
		else
		{
			#define _MAX_LEN (9 + _MAX_ATTR_SIZE)
			uint8_t buf[_MAX_LEN];
			uint8_t len;
			len = serialize(buf, _MAX_LEN, "8:1:", (uint64_t)_sys_time(), attr_num);
			len += _attr_get(objoffset, attr_num, buf+len);
			send_to_interested(objoffset, ca, offset2port(objoffset), MSG_ATTR_RET, buf, len );
		}
    }
}


__attribute__((weak)) uint16_t isGT(uint8_t attr_num1, uint8_t attr_num2)
{return 0;}

