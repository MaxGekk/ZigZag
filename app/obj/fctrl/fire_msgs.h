#ifndef _FIRE_MSGS_H_
#define _FIRE_MSGS_H_

#define FSTR_NWK_MSGS          "Сообщения:"
#define FSTR_NO_NWK_MSGS       "  -- Нет сообщений --"

#define FSTR_NODE_NOCFG        "Узел несконфигурирован"
#define FSTR_NODE_NOZONE       "Узел не приписан к зоне"
#define FSTR_ZONE_NODE_FMT     "з%u, н%u "
#define FSTR_NODANGER          "Норма."
#define FSTR_NOFIRE            "Нет пожара"
#define FSTR_FIREDANGER        "Опасность пожара!"
#define FSTR_FIRE              "Пожар!"
#define FSTR_NOPRECLEAN        "Датчик не требует чистки"
#define FSTR_PRECLEAN          "Датчик загрязняется"
#define FSTR_SENSOROK          "Датчик в норме"
#define FSTR_PREFAILURE        "Проверь датчик!"
#define FSTR_CLEAN_SENSOR      "Очисть датчик!"
#define FSTR_FAILURE           "Отказ датчика!"
#define FSTR_NOTHRESHOLDS      "Нет изменений"
#define FSTR_ALARM             "Тревога!"
#define FSTR_NOALARM           "Норма"

#define FSTR_NODE_ADD_FMT      "+Узел(0x%x, 0x%llx) (н%u)"
#define FSTR_NODE_ADD1_FMT     "+Узел: з%u, н%u"
#define FSTR_NODE_ATCAP_FMT    "Ошибка: +Узел(0x%x, 0x%llx) - нет памяти"
#define FSTR_NODE_NOTFOUND_FMT "Ошибка: +Узел(0x%x, 0x%llx) - не найден"
#define FSTR_NODE_DEL_FMT      "-Узел(0x%x, 0x%llx) (н%u)"
#define FSTR_NODE_DEL1_FMT     "-Узел: з%u, н%u"
#define FSTR_NODE_DEL_FMT1     "-Узел(0x%llx)"

#define FSTR_YES               "Да"
#define FSTR_NO                "Нет"
#define FSTR_SET               "Уст."
#define FSTR_CLEAR             "Сброс"
#define FSTR_STAT_YES          " Да  "
#define FSTR_STAT_NO           " Нет"

#define FSTR_MNU_DEVDATA       "Параметры устройства"
#define FSTR_MNU_ZONES         "Зоны"
#define FSTR_MNU_DATETIME      "Дата/время"
#define FSTR_MNU_FIRMWARE      "Версия ПО"
#define FSTR_MNU_SERIAL        "Серийный номер"
#define FSTR_MNU_NETWORK       "Беспроводная сеть"
#define FSTR_MNU_MSGS          "Сообщения"
#define FSTR_MNU_NODELIST      "Подключенные узлы"

#define FSTR_MNU_LASTMSG       "Последнее сообщение"

#define FSTR_MNU_FIRMWARE_VER  "Версия ПО: 0.3"
#define FSTR_MNU_SERIAL_NUM    "Серийный номер устройства: 0001"
#define FSTR_MNU_CONFIG_ERR    "Ошибка в конфигурации"
#define FSTR_MNU_NODE_NOTCONF  "Узел не найден"
#define FSTR_MNU_ZONE_ON       " (активна)"
#define FSTR_MNU_ZONE_OFF      " (неактивна)"
#define FSTR_MNU_ZONE          "Зона: "
#define FSTR_MNU_CONNECTED     "подключен"
#define FSTR_MNU_NOTCONNECTED  "неподключен"
#define FSTR_MNU_NONE          "<нет>"
#define FSTR_MNU_ATTENTION     "Внимание!"
#define FSTR_MNU_DATETIME_SET  "Установка даты и времени"



#endif // _FIRE_MSGS_H_

