#include "aux_task.h"
#include "util.h"
#include "adc.h"

#ifndef USE_LEGACY_CMSIS_RTOS
#include "rtx_os.h"
#endif

osThreadId_t aux_task_id;
#ifndef USE_LEGACY_CMSIS_RTOS
static uint32_t s_aux_thread_cb[WORDS(sizeof(osRtxThread_t))];
static uint64_t s_aux_task_stack[AUX_TASK_STACK / sizeof(uint64_t)];
static const osThreadAttr_t k_aux_thread_attr = {
        .name = "aux",
        .cb_mem = s_aux_thread_cb,
        .cb_size = sizeof(s_aux_thread_cb),
        .stack_mem = s_aux_task_stack,
        .stack_size = sizeof(s_aux_task_stack),
        .priority = AUX_TASK_PRIORITY,
    };

static uint32_t s_timer_100ms_cb[WORDS(sizeof(osRtxTimer_t))];
static const osTimerAttr_t k_timer_100ms_attr = {
        .name = "100ms",
        .cb_mem = s_timer_100ms_cb,
        .cb_size = sizeof(s_timer_100ms_cb),
    };
#endif

void timer_task_100mS(void * arg)
{
    osThreadFlagsSet(aux_task_id, FLAGS_AUX_100MS);
}

void create_aux_task(void)
{
    ADC_DAP_Initialize();
#ifndef USE_LEGACY_CMSIS_RTOS
    aux_task_id = osThreadNew(aux_task, NULL, &k_aux_thread_attr);
#else
    osThreadNew(aux_task, NULL, NULL);
#endif
}

void aux_task(void *arg)
{
    uint16_t flags = 0;
#ifdef USE_LEGACY_CMSIS_RTOS
    aux_task_id = osThreadGetId();
    osTimerId_t tmr_id = osTimerNew(timer_task_100mS, osTimerPeriodic, NULL, NULL);
#else
    osTimerId_t tmr_id = osTimerNew(timer_task_100mS, osTimerPeriodic, NULL, &k_timer_100ms_attr);
#endif
    osTimerStart(tmr_id, 3);
    while (1)
    {
        flags = osThreadFlagsWait(FLAGS_AUX_100MS, osFlagsWaitAny, osWaitForever);

        if (flags & FLAGS_AUX_100MS)
        {
            update_ADC_value();
        }
    }
}
