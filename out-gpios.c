#include "out-gpios.h"

#include "out-gpios.h"

uint16_t pulse_out_sequence1[4] = {
    0,
    1 << FIRST_OUT_PULSE,
    1 << FIRST_OUT_PULSE | 1 << SECOND_OUT_PULSE,
    1 << SECOND_OUT_PULSE 
};

uint16_t pulse_out_sequence2[4] = {
    0,
    1 << THIRD_OUT_PULSE,
    1 << THIRD_OUT_PULSE | 1 << FOURTH_OUT_PULSE,
    1 << FOURTH_OUT_PULSE 
};

uint8_t pulse_seq_index1 = 0;
bool reverse1 = false;
uint8_t pulse_seq_index2 = 0;
bool reverse2 = false;

static void init_gpio_as_output(uint8_t num) {
    gpio_set_function(num, GPIO_FUNC_SIO);
    gpio_set_dir(num, true);
}

void init_out_gpios() {
    init_gpio_as_output(FIRST_OUT_PULSE);
    init_gpio_as_output(SECOND_OUT_PULSE);
    init_gpio_as_output(THIRD_OUT_PULSE);
    init_gpio_as_output(FOURTH_OUT_PULSE);
}