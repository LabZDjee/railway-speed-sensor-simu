/**
 * Copyright (c) 2024 Gerard Gauthier
 * 
 * Preleminary tests
 * Just kept for reference:
 *  because this method cannot have a fine-grained control over
 *  frequency shifts (i.e. change behavior whithin a 1/4 signal period)
 */

#include "timer-managed.h"

#include "out-gpios.h"

// dHz
uint32_t freq1 = 70000;
uint32_t freq2 = 30000;

static bool timer1_callback(repeating_timer_t *rt) {
    SET_OUTPUT_PULSE_1
    return true; // keep repeating
}

static bool timer2_callback(repeating_timer_t *rt) {
    SET_OUTPUT_PULSE_2
    return true;
}

void setTimer(uint8_t num, uint32_t freq) {
    static bool is_init[2] = {false, false};
    static repeating_timer_t timer[2];
    int32_t delay = -10000000L/((int32_t)freq*4L);

    num = num & 1;
    if(is_init[num]) {
        cancel_repeating_timer(timer+num);
    } else {
        is_init[num] = true;
    }
  add_repeating_timer_us(delay, num==0 ? timer1_callback : timer2_callback, NULL, timer+num);
}
