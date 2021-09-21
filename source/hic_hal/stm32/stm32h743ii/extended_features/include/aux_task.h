#ifndef AUX_TASK_H_
#define AUX_TASK_H_

#include "cmsis_os2.h"

#define AUX_TASK_STACK (800)
#define AUX_TASK_PRIORITY (osPriorityNormal)
#define FLAGS_AUX_100MS         (1 << 0)

void create_aux_task(void);

void aux_task(void *arg);

#endif
