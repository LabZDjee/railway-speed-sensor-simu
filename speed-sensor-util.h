#ifndef SPEED_SENSOR_UTIL_H
#define SPEED_SENSOR_UTIL_H

#include <stdint.h>
#include <stdbool.h>

#include "speed-sensor.h"

typedef enum {
    e_syntax_error,
    e_help,
    e_extended_help,
    e_execute_list,
    e_new_value,
    e_new_record,
    e_init_list,
    e_print_list,
    e_close_list,
    e_new_speed_definition,
    e_range_error,
    e_empty
} command_e;

#define MIN_N_TEETH 13
#define MAX_N_TEETH 1908
#define MIN_DIA_MM 300
#define MAX_DIA_MM 1500
#define MIN_RATIO 0.001f
#define MAX_RATIO 4.0f

#define PI 3.141592653589793f

#define CTRL_E_ASCII  5
#define BACK_SPACE_ASCII 8

// given a speed in km/h, returns frequency in Hz
// resource (read only): speed_definition
float get_frequency(float speed);
// given a frequency in Hz, returns period (in µs)
// note: this period is four times shorter than 1/freq
uint32_t get_period(float freq);
// given a period as defined above, return either
// speed in km/h (if function value_is_speed return true) or frequency in Hz
float get_corrected_value(uint32_t period);
// tells if speed_definition is correct so values are defined
// in the realm of speed (km/h) instead of frequency (in Hz)
bool value_is_speed();
// tells if definitions for both sensors are equal
bool are_sensors_equal(const sequence_values_t*);
// call printf to output syntax of commands
void print_help(bool extended);
// return "+", "-", "+-", "-+" depending on 'reverse' values for sensors
const char* get_reverse_description(const sequence_values_t* seq);

// takes an input from user
// scans it and returns what scan brought
// depending on correct command found
// possibly alters external variable cur_values or speed_definition
command_e process_input(const char * input);

// return true if both strings have same contents
bool are_strings_equal(const char* s1, const char* s2);
// returns buffer filled with value with sensible decimals depending on its magnitude
// depending on value, be careful to provision enough size for buffer
// this low level function is not safe in that respect
char* _unsafe_format_float(float value, char* buffer);
// removes spaces (and tabs) from start and end of a string
// returns str address
char* str_trim(char *str);

// get a string from stdin, cr/lf terminated (excluded)
// back space character is managed
// return false in case of buffer overflow
bool get_input(char* buffer, uint16_t buffer_size);

// simply flushes all pending characters from standard input
void flush_stdin();

#endif