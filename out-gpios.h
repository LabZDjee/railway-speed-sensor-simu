#ifndef OUT_GPIOS_H
#define OUT_GPIOS_H

#include "pico/stdlib.h"

// outputs GPIO numbers for sensor 1, channels A and B and sensor 2, channels A and B 
#define FIRST_OUT_PULSE  1
#define SECOND_OUT_PULSE 0
#define THIRD_OUT_PULSE  4
#define FOURTH_OUT_PULSE 3

// whether signal should be reversed for sensors 1 and 2
extern bool reverse1;
extern bool reverse2;

// makes sensor 1 output progress one step (4 steps = 1 cycle)
#define SET_OUTPUT_PULSE_1 {\
  gpio_put_masked(1<<FIRST_OUT_PULSE | 1 << SECOND_OUT_PULSE, pulse_out_sequence1[pulse_seq_index1]); \
  if(reverse1) { \
        pulse_seq_index1 = (pulse_seq_index1-1) & 3; \
    } else { \
        pulse_seq_index1 = (pulse_seq_index1+1) & 3; \
    }}

// makes sensor 2 output progress one step (4 steps = 1 cycle)
#define SET_OUTPUT_PULSE_2 {\
  gpio_put_masked(1<<THIRD_OUT_PULSE | 1 << FOURTH_OUT_PULSE, pulse_out_sequence2[pulse_seq_index2]); \
  if(reverse2) { \
        pulse_seq_index2 = (pulse_seq_index2-1) & 3; \
    } else { \
        pulse_seq_index2 = (pulse_seq_index2+1) & 3; \
    }}

// used by macros above (shouldn't be accessed elsewhere)
extern uint16_t pulse_out_sequence1[4];
extern uint8_t pulse_seq_index1;
extern uint16_t pulse_out_sequence2[4];
extern uint8_t pulse_seq_index2;

// required to be called one time before actual usage of the module
void init_out_gpios();

#endif