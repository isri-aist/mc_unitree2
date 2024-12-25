#pragma once

#define RT_PRIO_MAX   255
#define TASK_PERIOD  1000 // ns


#ifdef __cplusplus
extern "C" {
#endif
int set_sched_prio(short, unsigned long);
#ifdef __cplusplus
}
#endif
