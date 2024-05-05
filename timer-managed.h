#ifndef TIMER_MANAGED_H
#define TIMER_MANAGED_H

#include "pico/stdlib.h"

// dHz
#define MAX_FREQ 70000
extern uint32_t freq1;
extern uint32_t freq2;

void setTimer(uint8_t num, uint32_t freq);

#endif