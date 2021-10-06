/*
 *  Description:
 *    Create an auxiliary task so that we don't need to put UDB's logic in the main task.
 *
 */
#ifndef _UDB_EXTENDED_FEATURES_TASK_INCLUDED_H_
#define _UDB_EXTENDED_FEATURES_TASK_INCLUDED_H_

#include "cmsis_os2.h"

#define UDB_EXTENDED_FEATURES_TASK_STACK          (800)
#define UDB_EXTENDED_FEATURES_TASK_PRIORITY       (osPriorityNormal)
#define UDB_EXTENDED_FEATURES_TASK_FLAGS_3S       (1 << 0)

// 1 tick is 10ms
#define UDB_EXTENDED_FEATURES_TASK_TICKS_3S       (300)

void udb_extended_features_task_create(void);

#endif // _UDB_EXTENDED_FEATURE_TASK_INCLUDED_H_
