#ifndef __TASK_H__
#define __TASK_H__

// 每次调用时，若距上次执行不足 T 毫秒则直接返回
#define PERIODIC(T) \
    static uint32_t next_time = 0; \
    if (HAL_GetTick() < next_time) return; \
    next_time += (T);

#define PERIODIC_START(NAME, T) \
    static uint32_t NAME##_next_time = 0; \
    if (HAL_GetTick() >= NAME##_next_time) { \
        NAME##_next_time += (T);

#define PERIODIC_END \
    }

#endif