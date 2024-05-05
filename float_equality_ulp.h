#ifndef _FLOAT_EQUALITY_ULP_H_
#define _FLOAT_EQUALITY_ULP_H_

#include <stdbool.h>
#include <stdint.h>

// Uses the ULP (Unit in the Last Place) technique
// See https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition

// tests if floats are equal
bool are_floats_equal_ulp(float f1, float f2);

// sets the maximum 24-bit mantissa difference between two single precision (32 bits) floats
// as defined in IEE 754 to consider those floats as equal
// very small values like 1 or 2 are considered enough
// if not set, the max_ulp is already set at a reasonable value
// now, if not many significant digits are acceptable (4 or 5),
// a max_ulp of 100 or 1000 can be considered
void set_ulp_for_float_equality(uint16_t new_max_ulp);


#endif