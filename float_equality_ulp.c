
#include "float_equality_ulp.h"

#include <stdlib.h>

#define do_not_DEBUG

#ifdef DEBUG
 #include <stdio.h>
#endif

// Uses the ULP (Unit in the Last Place) technique
// See https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition

static uint16_t max_ulp = 2;

void set_ulp_for_float_equality(uint16_t new_max_ulp) {
    max_ulp = new_max_ulp;
}

bool are_floats_equal_ulp(float f1, float f2) {
    const int32_t f1_as_bits = *(int32_t*)&f1;
    const int32_t f2_as_bits = *(int32_t*)&f2;

    if(f1_as_bits < 0 != f2_as_bits < 0) {
  #ifdef DEBUG
    printf("near zero with opposite signs: %.10f vs %.10f\n", f1, f2);
  #endif
        return f1 == f2;
    }
  #ifdef DEBUG
    printf("ulp=%d (max_ulp=%hu)\n", (int)abs(f1_as_bits - f2_as_bits), max_ulp);
  #endif
    return abs(f1_as_bits - f2_as_bits) <= max_ulp;
}