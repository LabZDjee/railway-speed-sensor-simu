#ifndef SENSOR_SENSOR_H
#define SENSOR_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define SEQUENCE_VALUE_ARRAY_SIZE 64

#define STEPS_PER_SECOND 5
#define TIMER_COUNT_PER_STEP 2

#define NB_ARMED_COUNTS 1

extern volatile uint8_t timer_armed_counts[NB_ARMED_COUNTS];

#define WAIT_FOR_FLAG(n, v) { timer_armed_counts[n]=v; while(timer_armed_counts[n]!=0){} }

typedef struct {
    float firstValue;
    float secondValue;
    bool firstReverse;
    bool secondReverse;
    uint16_t delay;
} sequence_values_t;

typedef struct {
    uint16_t n_teeth;
    uint16_t diameter_mm;
    float gear_ratio;
} speed_definition_t;

#define MIN_FREQUENCY 0.1f
#define MAX_FREQUENCY 7300.0f

extern sequence_values_t next_values;
extern speed_definition_t speed_definition;

#endif