#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "float_equality_ulp.h"
#include "speed-sensor.h"

#include "speed-sensor-util.h"

#define NO_REVSERSE_DEFINED "=="

static bool test_reverse(const char* str) {
    if(are_strings_equal(str, NO_REVSERSE_DEFINED)) // very likey case nothing was found in input
     return true;
    if(are_strings_equal(str, "+") || are_strings_equal(str, "++")) {
        next_values.firstReverse = false;
        next_values.secondReverse = false;
        return true;
    }
    if(are_strings_equal(str, "+-")) {
        next_values.firstReverse = false;
        next_values.secondReverse = true;
        return true;
    }
    if(are_strings_equal(str, "-+")) {
        next_values.firstReverse = true;
        next_values.secondReverse = false;
        return true;
    }
    if(are_strings_equal(str, "-") || are_strings_equal(str, "--")) {
        next_values.firstReverse = true;
        next_values.secondReverse = true;
        return true;
    }
    return false;
}

char* _unsafe_format_float(float value, char* buffer) {
    float absValue = fabs(value);
    if(absValue<10.0f) {
        sprintf(buffer, "%.4f", value);
    } else if(absValue<100.0f) {
        sprintf(buffer, "%.2f", value);
    } else if(absValue<1000.0f) {
        sprintf(buffer, "%.1f", value);
    } else {
        sprintf(buffer, "%.0f", value);
    }
    return buffer;
}

float get_frequency(float speed) {
    speed = fabsf(speed);
    if(!value_is_speed()) {
        return speed;
    }
    return speed * speed_definition.gear_ratio * speed_definition.n_teeth / (PI * 3.6f / 1000.0f * speed_definition.diameter_mm);
}

uint32_t get_period(float freq) {
    if(freq == 0.0f) {
        return 0;
    }
    if(freq < MIN_FREQUENCY) {
        freq = MIN_FREQUENCY;
    } else if (freq > MAX_FREQUENCY) {
        freq = MAX_FREQUENCY;
    }
    return round((1000000.0f/4.0f) / (float)freq);
}

float get_corrected_value(uint32_t period) {
    if(period==0) {
        return 0;
    }
    if(!value_is_speed()) {
        return (1000000.0f/4.0f) / (float)period; 
    }
    return PI * 3.6 * 1000000.0f / 4.0f / 1000.0f * speed_definition.diameter_mm /  (speed_definition.n_teeth * speed_definition.gear_ratio * (float)period);
}

bool value_is_speed() {
    return speed_definition.diameter_mm != 0 && speed_definition.n_teeth != 0;
}

bool are_sensors_equal(const sequence_values_t* pSeq) {
 if(pSeq == NULL) {
    return false;
 }
 return
    pSeq->firstReverse == pSeq->secondReverse &&
    are_floats_equal_ulp(pSeq->firstValue, pSeq->secondValue);
}

bool are_strings_equal(const char* s1, const char* s2) {
    return strcmp(s1, s2) == 0;
}

char *str_trim(char *str) {
  char* start = str;
  // trim leading spaces
  while(isspace((int)*str)) {
    str++;
  } 
  if(*str != '\0') {
    // trim trailing spaces
    char* end = str + strlen(str) - 1;
    while(end > str && isspace((int)*end)) {
        end--;
    }
    // writes new null terminator character
    end[1] = '\0';
  }  
  memmove((void*)start, (const void*)str, strlen(str)+1);
  return start;
}

void print_help(bool extended) {
    const char* short_help_txt =    " {value1}[:{value2}][{rev_defs}] define immediate values\n"
     " {rev_defs} defines moving directions (forward/reverse)\n"
     " {delay}\"[>{value1}[:{value2}][{revDefs}]] new sequence item\n"
     " ( start sequence\n"
     " ) end sequence\n"
     " ![!] execute sequence, infinite loop\n"
     " !? print sequence\n"
     " {n_teeth},{dia_mm}[,{ratio}] define speed to frequency parameters\n"
     " ?[?] help, extended help\n";

    if(!extended) {
        printf(short_help_txt);
        return;
    }
    printf(
     "Syntax of commands for speed sensor simulator:\n"
     "%s"
     "  with {value1}, {value2} define speed (or frequency)\n"
     "       If only one defined, define both as equal\n"
     "       {rev_defs} + (both forward), -+ (first reverse only),\n"
     "        +- (second reverse only), - (both reversed)\n"
     "       {delay} timer value in seconds\n"
     "       {n_teeth} number of teeth [%d, %d], 0 to disable frequency\n"
     "       Calculations done according to speed:\n"
     "        {diam_mm} diameter in millimeters [%d mm, %d mm]\n"
     "        {ratio} gear ratio [%.4f, %.1f], defaults to 1.0\n"
     "Notes:\n"
     " no limit on values imposed, frequencies are clamped to [%.1f Hz, %.0f Hz]\n"
     " value of zero always indicates frequency and speed are null\n"
     " when speeds are considered, they are in km/h (and frequencies are in Hz)\n"
     " ^c: cancels current sequence, ^e: toggles character echo,\n"
     " empty command: details of current state\n",
     short_help_txt, MIN_N_TEETH, MAX_N_TEETH, MIN_DIA_MM, MAX_DIA_MM, MIN_RATIO, MAX_RATIO, MIN_FREQUENCY, MAX_FREQUENCY);
}

command_e process_input(const char * input) {
 float f1, f2;
 int d1, d2;
 char str[256] = NO_REVSERSE_DEFINED;
 if(strlen(input) == 0)  {
    return e_empty;
 }
 if(sscanf(input, "%d\">%f:%f%s", &d1, &f1, &f2, str)>=3) {
    if(!test_reverse(str)) {
     return e_syntax_error;
    }
    next_values.firstValue = f1;
    next_values.secondValue = f2;
    next_values.delay = d1;
    return e_new_record;
 }
 strcpy(str, NO_REVSERSE_DEFINED);
 if(sscanf(input, "%d\">%f%s", &d1, &f1, str)>=2) {
    if(!test_reverse(str)) {
     return e_syntax_error;
    }
    next_values.firstValue = f1;
    next_values.secondValue = f1;
    next_values.delay = d1;
    return e_new_record;
 }
 strcpy(str, NO_REVSERSE_DEFINED);
 if(sscanf(input, "%d\">%s", &d1, str)==2) {
    if(!test_reverse(str)) {
     return e_syntax_error;
    }
    next_values.delay = d1;
    return e_new_record;
 }
 strcpy(str, NO_REVSERSE_DEFINED);
 if(sscanf(input, "%d%s", &d1, str)==2 && 
    (are_strings_equal(str, "\"") || are_strings_equal(str, "\">"))) {
    next_values.delay = d1;
    return e_new_record;
 }
 if(are_strings_equal(input, "(")) {
    return e_init_list;
 }
 if(are_strings_equal(input, ")")) {
    return e_close_list;
 }
 if(are_strings_equal(input, "!")) {
    return e_execute_list;
 }
 if(are_strings_equal(input, "!!")) {
    return e_loop_list;
 }
 if(are_strings_equal(input, "!?")) {
    return e_print_list;
 }
 if(are_strings_equal(input, "?")) {
    return e_help;
 }
 if(are_strings_equal(input, "??")) {
    return e_extended_help;
 }
 f1 = 1.0;
 if(sscanf(input, "%d,%d,%f", &d1, &d2, &f1)>=2) {
    if(d1 != 0 && d2 != 0) {
        if(d1 < MIN_N_TEETH || d1 > MAX_N_TEETH ||
           d2 < MIN_DIA_MM || d2 > MAX_DIA_MM ||
           f1 < MIN_RATIO  || f1 > MAX_RATIO) {
            return e_range_error;
        }
    } else {
        d1 = d2 = 0;
        f1 = 1.0;
    }
    speed_definition.n_teeth = d1;
    speed_definition.diameter_mm = d2;
    speed_definition.gear_ratio = f1;
    return e_new_speed_definition;
 }
 strcpy(str, NO_REVSERSE_DEFINED);
 if(sscanf(input, "%f:%f%s", &f1, &f2, str)>=2) {
    if(!test_reverse(str)) {
     return e_syntax_error;
    }
    next_values.firstValue = f1;
    next_values.secondValue = f2;
    return e_new_value;
 }
 strcpy(str, NO_REVSERSE_DEFINED);
 if(sscanf(input, "%f%s", &f1, str)>=1) {
    if(!test_reverse(str)) {
     return e_syntax_error;
    }
    next_values.firstValue = f1;
    next_values.secondValue = f1;
    return e_new_value;
 }
 if(test_reverse(input)) {
    return e_new_value;
 }
 return e_syntax_error;
}

bool get_input(char* buffer, uint16_t buffer_size) {
    static bool echo = true;
    uint16_t index = 0;
    int ch;
    if(buffer_size<1) {
        return false;
    }
    while(index < buffer_size-2) {
        ch = getchar();
        if(ch == CTRL_E_ASCII) {
          echo = !echo;
        } else if(ch == BACK_SPACE_ASCII) {
            if(index>0) {
                index--;
                printf("%c %c", BACK_SPACE_ASCII, BACK_SPACE_ASCII);
            }
        } else if (ch == '\r' || ch == '\n') {
            buffer[index] = '\0';
            if(ch == '\r') {
                ch = getchar_timeout_us(200000);
                if(ch >=0 && ch != '\n') {
                    ungetc(ch, stdin);
                }
            }
            return true;
        } else {
            buffer[index++] = ch;
            if(echo) {
                putchar(ch);
            }
        }
    }
    buffer[index] = '\0';
    return false;
}

static const char* _reverse_description_[] = {"+", "-", "+-", "-+"};

const char* get_reverse_description(const sequence_values_t* seq) {
    if(seq==NULL) {
        return "";
    }
    bool fr = seq->firstReverse, sr = seq->secondReverse;
    if(fr == sr) {
        return _reverse_description_[fr ? 1 : 0];
    }
    return _reverse_description_[fr ? 3: 2];
}

void flush_stdin() {
    while(getchar_timeout_us(0) >= 0) {
    }
}