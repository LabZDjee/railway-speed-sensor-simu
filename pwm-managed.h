#ifndef PMW_MANAGED_H
#define PWM_MANAGED_H

#include "pico/stdlib.h"
#include "pico/util/queue.h"

typedef struct {
    uint32_t max_count1;
    uint32_t max_count2;
    bool invert1;
    bool invert2;
} intercore_data_t;

extern queue_t call_queue;

void start_pwm();
void core1_main();

/*
 * Those counts directly influence sensor outputs
 *  1 and 2 refer to sensors 1 and 2 
 *  unit of this counter is 1 µS
 *  max_cycle_count represents 1/4 of sensor cycle period
 */
// for sensor 1
extern uint32_t cycle_count1;
extern uint32_t max_cycle_count1;
// for sensor 2
extern uint32_t cycle_count2;
extern uint32_t max_cycle_count2;

#endif