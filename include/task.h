#ifndef     _TASK_H
#define     _TASK_H

/*! @file   task.h
 *  @brief  Функции запуска задач
 *  @author Max Gekk
 *  @date   июль 2007
 *  @version 1 
 */

/*! @defgroup init Инициализация системы @{ */

/*! @fn void __sys_init();
 *  @brief Инициализация прикладной части ZigZag-a
 * */
void __app_init();
/*! @} */

/*! @defgroup task Запуск функции в задаче @{ */

typedef void    (* task_ft)();

/*! @fn void __post_task( task_ft task_func );
 *  @brief Запуск задачи
 * */
int16_t __post_task( task_ft task_func );
/*! @} */

#endif  /*  _TASK_H */
