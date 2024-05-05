/**
 * Copyright (c) 2024 Gerard Gauthier
 *
 * FTU3 speed sensor simulator of two independent sensors with A and B channels
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "float_equality_ulp.h"
#include "pwm-managed.h"
#include "speed-sensor-util.h"

#include "speed-sensor.h"

#define no_DEBUG_STUFF

enum {
    es_default,
    es_recording
} state_machine = es_default;

#ifdef DEBUG_STUFF

typedef struct  {
    int value;
    const char* str;
} enum_as_str_ele;

enum_as_str_ele cmd_result_as_str_map[] = {
    {e_help, "help"},
    {e_extended_help, "extended_help"},
    {e_execute_list, "execute_list"},
    {e_print_list, "print_list"},
    {e_new_value, "new_value"},
    {e_new_record, "new_record"},
    {e_init_list, "init_list"},
    {e_close_list, "close_list"},
    {e_new_speed_definition, "speed_definition"},
    {e_syntax_error, "syntax_error"},
    {e_range_error, "range_error"},
    {e_empty, "empty_command"},
    {9999, NULL}
};

enum_as_str_ele state_machine_as_str_map[] = {
    {es_default, "default"},
    {es_recording, "recording"}
    {9999, NULL}
};

static const char* cmd_result_as_str(int v, const enum_as_str_ele* pDef) {
    int i;
    if(pDef !=NULL) {
        for(i=0; pDef[i].str != NULL; i++) {
            if(pDef[i].value == v) {
            return pDef[i].str;
            }
        }
    }
    return("UNKNOWN!!!");
}

#endif

const char* msg_sequence_interrupted = "^C - sequence interrupted!\n";

const char* truthness_as_str(bool v) {
    return v ? "true" : "false";
}

// those two definitions will control a step current => next
// delay of a step is controlled by next_values.delay (in seconds)
static sequence_values_t current_values;
sequence_values_t next_values = {0, 0, false, false, 0}; // starts at 0 Hz, forward

speed_definition_t speed_definition =
   {0, 0, 1.0}; // init with no definition of speed which is not managed (frequency instead)

// inter-core queue
queue_t call_queue;

static sequence_values_t sequence_array[SEQUENCE_VALUE_ARRAY_SIZE];
static uint8_t sequence_index = 0;

// LED cycle
static volatile uint8_t max_led_repeat = 10;

// value(s) automatically decremented (and stuck at zero) every timer_callback call
volatile u_int8_t timer_armed_counts[NB_ARMED_COUNTS]; // byte: this is atomic by design

static const uint LED_PIN = PICO_DEFAULT_LED_PIN;

static bool timer_callback(repeating_timer_t *rt) {
    static uint8_t repeat_for_led = 0;
    uint8_t i;
    for(i = 0; i < NB_ARMED_COUNTS; i++) {
        if(timer_armed_counts[i] != 0) {
            timer_armed_counts[i]--;
        }
    }
    if (repeat_for_led == 0) {
        gpio_put(LED_PIN, 1);
    } else if (repeat_for_led == 1) {
        gpio_put(LED_PIN, 0);
    }
    if (++repeat_for_led >= max_led_repeat) {
        repeat_for_led = 0;
    }
    return true; // keep repeating
}

// adjust LED frequency according to actual sensor frequency
static void set_led_repeat(float frequency) {
    if(frequency < 0.5f) {
        max_led_repeat = 20;
    } else if(frequency < 2.0f) {
        max_led_repeat = 15;
    } else if(frequency < 3.0f) {
        max_led_repeat = 16;
    } else if(frequency < 15.0f) {
        max_led_repeat = 14;
    } else if(frequency < 22.0f) {
        max_led_repeat = 12;
    } else if(frequency < 40.0f) {
        max_led_repeat = 10;
    } else if(frequency < 200.0f) {
        max_led_repeat = 9;
    } else if(frequency < 400.0f) {
        max_led_repeat = 8;
    } else if(frequency < 600.0f) {
        max_led_repeat = 7;
    } else if(frequency < 1000.0f) {
        max_led_repeat = 6;
    } else if(frequency < 1200.0f) {
        max_led_repeat = 5;
    } else if(frequency < 1750.0f) {
        max_led_repeat = 4;
    } else if(frequency < 2000.0f) {
        max_led_repeat = 3;
    } else {
        max_led_repeat = 2;
    } 
}

// index of timer which manages sequences
#define TIMER_SEQ_ID 0

static intercore_data_t inter_core_data = {0, 0, false, false};

// update delays and forward/reverse ways
// only done if required
static void send_intercore_data(intercore_data_t* p_new_intercore_data) {
    if(p_new_intercore_data == NULL) {
        queue_add_blocking(&call_queue, &inter_core_data);
        return;
    }
    if(p_new_intercore_data == NULL ||
       p_new_intercore_data->max_count1 != inter_core_data.max_count1 ||
       p_new_intercore_data->max_count2 != inter_core_data.max_count2 ||
       p_new_intercore_data->invert1 != inter_core_data.invert1 ||
       p_new_intercore_data->invert2 != inter_core_data.invert2) {
        inter_core_data = *p_new_intercore_data;
        queue_add_blocking(&call_queue, &inter_core_data);
    }
}

// manages a new step: update actual state of frequency
// and display step information (if required)
// in: 
//  display_information => 0: none, 1: only one speed/frequency, 2: both
//  delay_elpased: current delay
// out: false if interrupt required
static bool __timer_controlled_sequence_step_actions(uint8_t display_information, int16_t delay_elapsed) {
    float f1, f2;
    intercore_data_t temp_intercore_data;
    char buf1[16], buf2[16];
    int ch;

    f1 = get_frequency(current_values.firstValue);
    f2 = get_frequency(current_values.secondValue);
    set_led_repeat(fmaxf(f1, f2));
    temp_intercore_data.invert1 = current_values.firstReverse;
    temp_intercore_data.invert2 = current_values.secondReverse;
    temp_intercore_data.max_count1 = get_period(f1);
    temp_intercore_data.max_count2 = get_period(f2);
    send_intercore_data(&temp_intercore_data);
    ch = getchar_timeout_us(0);
    if(ch == 3) { // Ctrl-C
        return false;
    } else if (ch >= 0) {
        ungetc(ch, NULL);
    } 
    if(display_information == 0) {
        return true;
    }
    if(delay_elapsed >= 0) {
        printf("\r%ld\" - ", delay_elapsed);
    } else {
        printf("\r");
    }
    if(value_is_speed()) {
        f1 = get_corrected_value(temp_intercore_data.max_count1);
        f2 = get_frequency(f1);
        printf("Actual speed %c%s km/h (%s Hz)",
               temp_intercore_data.invert1?'-':'+',
               _unsafe_format_float(f1, buf1),
               _unsafe_format_float(f2, buf2));
        if(display_information == 2) {
            f1 = get_corrected_value(temp_intercore_data.max_count2);
            f2 = get_frequency(f1);
            printf(" : %c%s km/h (%s Hz)",
               temp_intercore_data.invert2?'-':'+',
               _unsafe_format_float(f1, buf1),
               _unsafe_format_float(f2, buf2));
        }
    } else {
       f1 = get_corrected_value(temp_intercore_data.max_count1);
       printf("Actual frequency %c%s Hz",
               temp_intercore_data.invert1?'-':'+',
               _unsafe_format_float(f1, buf1));
       if(display_information == 2) {
            f1 = get_corrected_value(temp_intercore_data.max_count2);
            printf(" : %c%s Hz",
               temp_intercore_data.invert2?'-':'+',
               _unsafe_format_float(f1, buf1));
        }
    }
    printf("     ");
    return true;
}

// one sequence step from current_values to next_values
// in: resync, should be true only at the start of a new full sequence to ensure best timer synchronization 
// out: false if sequence interruption required
static bool timer_controlled_sequence_step(bool resync) {
    // displays one sensor value if one is consistent throughout the entire step
    const uint8_t display_nb_sensors =
                are_sensors_equal(&current_values) && are_sensors_equal(&next_values) ? 1 : 2; 
    if(resync) {
         WAIT_FOR_FLAG(TIMER_SEQ_ID, 1)
    }
    if(next_values.delay) { // edge case when delay is null: hopes directly to final value
        uint16_t i, nb_steps = STEPS_PER_SECOND * next_values.delay;
        // linear progression
        const float f1_step = (next_values.firstValue - current_values.firstValue) / nb_steps;
        const float f2_step = (next_values.secondValue - current_values.secondValue) / nb_steps;
        nb_steps--; // last step is skipped and managed out of loop
        for(i = 0; i < nb_steps; i++) {
            if(!__timer_controlled_sequence_step_actions(
                    i % STEPS_PER_SECOND == 0 ? display_nb_sensors : 0,
                    i / STEPS_PER_SECOND)) {
                printf("\n");
                return false;
            }
            current_values.firstValue += f1_step;
            current_values.secondValue += f2_step;
            WAIT_FOR_FLAG(TIMER_SEQ_ID, TIMER_COUNT_PER_STEP)
        }
    }
    // makes sure last step lands accuratley at next_values
    current_values = next_values;
    __timer_controlled_sequence_step_actions(display_nb_sensors,
                                             next_values.delay ? next_values.delay : -1);
    printf("\n");
    return true;
}

int main() {
    static repeating_timer_t timer;
    static char str[80], buf1[16], buf2[16];
    int i;
    float f;
  
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    add_repeating_timer_ms(-100, timer_callback, NULL, &timer);

    stdio_init_all();
    sleep_ms(4000); // better to ensure virtual serial port is properly set

    printf("Speed sensor simulator\n");
    printf("Hardware clock: %lu Hz\n", clock_get_hz(clk_sys));
    print_help(false);
   
    queue_init(&call_queue, sizeof(intercore_data_t), 2);

    multicore_launch_core1(core1_main);

    send_intercore_data(NULL); // init inter-core data
    current_values = next_values;
    timer_controlled_sequence_step(true); // set to whatever values current_values is initalized with

     while (true) {
        printf(">");
        get_input(str, sizeof(str));
        str_trim(str);
        if(strlen(str) == 0) { // void command: display current values
             bool are_sensors_equal = inter_core_data.invert1 == inter_core_data.invert2 &&
                                      inter_core_data.max_count1 == inter_core_data.max_count2;
            if(value_is_speed()) {
                *buf1 = '\0';
                if(!are_floats_equal_ulp(speed_definition.gear_ratio, 1.0)) {
                    sprintf(buf1, ", gear ratio: %f", speed_definition.gear_ratio);
                }
                printf("%hu teeth, wheel diameter: %hu mm%s\n", speed_definition.n_teeth, speed_definition.diameter_mm, buf1);
                f = get_corrected_value(inter_core_data.max_count1);
                printf("%c%s km/h (%s Hz)", inter_core_data.invert1 ? '-' : '+',
                                            _unsafe_format_float(f, buf1),
                                            _unsafe_format_float(get_frequency(f), buf2));
                if(!are_sensors_equal) {
                    f = get_corrected_value(inter_core_data.max_count2);
                    printf(" : %c%s km/h (%s Hz)", inter_core_data.invert2 ? '-' : '+',
                                                _unsafe_format_float(f, buf1),
                                                _unsafe_format_float(get_frequency(f), buf2));
                }
                printf("\n");
            } else {
                printf("No speed defined: only deals with frequencies\n");
                f = get_corrected_value(inter_core_data.max_count1);
                printf("%s Hz, %s", _unsafe_format_float(f, buf1),
                                    inter_core_data.invert1 ? "reversed" : "forward");
                if(!are_sensors_equal) {
                    f  = get_corrected_value(inter_core_data.max_count2);
                     printf(" : %s Hz, %s", _unsafe_format_float(f, buf1),
                                    inter_core_data.invert2 ? "reversed" : "forward");
                }
                printf("\n");
            }
            continue;
        }
        printf("\n");
        command_e r = process_input(str);
    #ifdef DEBUG_STUFF
        printf("result: %s, state: %s\n", cmd_result_as_str(r, cmd_result_as_str_map),
                                          cmd_result_as_str(state_machine, state_machine_as_str_map));
     #endif
        switch(r) {
            case e_empty:
               break;
            case e_new_record:
               if(state_machine != es_recording) {
                    goto _immediate_value;
               } else if(sequence_index == SEQUENCE_VALUE_ARRAY_SIZE){
                    printf("Error: recording array is full!\n");
               } else {
                    sequence_array[sequence_index++] = next_values;
    #ifdef DEBUG_STUFF
                    printf("%.2f%s:%.2f%s %hd\"\n", next_values.firstValue, next_values.firstReverse ? " rev" : "",
                            next_values.secondValue, next_values.secondReverse ? " rev" : "", next_values.delay);
    #endif
               }
               break;
            case e_new_value:
                next_values.delay = 0;
                current_values = next_values;
               _immediate_value:
    #ifdef DEBUG_STUFF
                printf("%.2f%s:%.2f%s %hd\"\n", next_values.firstValue, next_values.firstReverse ? " rev" : "",
                    next_values.secondValue, next_values.secondReverse ? " rev" : "", next_values.delay);
    #endif
                if(!timer_controlled_sequence_step(true)) {
                    printf(msg_sequence_interrupted);
                }
                flush_stdin();
                break;
            case e_new_speed_definition:
                if(speed_definition.n_teeth==0 || speed_definition.diameter_mm==0) {
                    printf("Speed definition cancelled (only deals with frequencies)\n");
                } else {
                    printf("New speed definition: %hd teeth, diameter= %hd mm, r=%f\n", speed_definition.n_teeth, speed_definition.diameter_mm, speed_definition.gear_ratio);
                }
                break;
            case e_syntax_error:
            case e_range_error:
                break;
            case e_init_list:
                sequence_index = 0;
                state_machine = es_recording;
                break;
            case e_close_list:
                state_machine = es_default; 
                break;
            case e_execute_list:
                state_machine = es_default;
                for(i=0; i<sequence_index; i++) {
                    next_values = sequence_array[i];
                    printf("Step %d", i+1);
                    if(next_values.delay) {
                        printf(" %hd\"", next_values.delay);
                    }
                    if(value_is_speed()) {
                       printf(" %s > %s km/h",
                              _unsafe_format_float(current_values.firstValue, buf1),
                              _unsafe_format_float(next_values.firstValue, buf2));
                       if(next_values.delay) {
                            f = (next_values.firstValue - current_values.firstValue) / (3.6 * (float)next_values.delay);
                            printf(" %s m/s2", _unsafe_format_float(f, buf1));
                       }
                    } else {
                       printf(" %s > %s Hz",
                              _unsafe_format_float(current_values.firstValue, buf1),
                              _unsafe_format_float(next_values.firstValue, buf2));
                    }
                    if(!are_floats_equal_ulp(current_values.secondValue, current_values.firstValue) ||
                       !are_floats_equal_ulp(next_values.secondValue, next_values.firstValue)) {
                        if(value_is_speed()) {
                            printf(" : %s > %s km/h",
                                _unsafe_format_float(current_values.secondValue, buf1),
                                _unsafe_format_float(next_values.secondValue, buf2));
                            if(next_values.delay) {
                                f = (next_values.secondValue - current_values.secondValue) / (3.6 * (float)next_values.delay);
                                printf(" %s m/s2", _unsafe_format_float(f, buf1));
                            }
                        } else {
                            printf(" : %s > %s Hz",
                                _unsafe_format_float(current_values.secondValue, buf1),
                                _unsafe_format_float(next_values.secondValue, buf2));
                        }
                    }
                    const char* pCurDesc = get_reverse_description(&current_values),
                              * pNextDesc = get_reverse_description(&next_values);
                    printf(" %s", pCurDesc);
                    if(pCurDesc != pNextDesc) {
                        printf( " > %s", pNextDesc);
                    }
                    printf("\n");
                    if(!timer_controlled_sequence_step(i == 0)) {
                        printf(msg_sequence_interrupted);
                        break;
                    }
                }
                break;
            case e_print_list:
                for(i=0; i<sequence_index; i++) {
                    printf("%i- %hu\"> %c%s : %c%s\n",
                           i+1, sequence_array[i].delay,
                           sequence_array[i].secondReverse?'-':'+',
                           _unsafe_format_float(sequence_array[i].firstValue, buf1),
                           sequence_array[i].secondReverse?'-':'+',
                           _unsafe_format_float(sequence_array[i].secondValue, buf2)
                           );
                }
                if(sequence_index == 0) {
                    printf("Sequence list is empty\n");
                } else {
                    printf("Values are in %s\n", value_is_speed() ? "km/h" : "Hz");
                }
                break;
            case e_help:
                print_help(false);
                break;
            case e_extended_help:
                print_help(true);
                break;
        }
    }
}

