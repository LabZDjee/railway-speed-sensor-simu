/**
 * Copyright (c) 2024 Gerard Gauthier
 */

#include "pwm-managed.h"

#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/critical_section.h"

#include "out-gpios.h"

#define SLICE_NUM 0

uint32_t cycle_count1 = 1;
uint32_t max_cycle_count1 = 0; // initially stopped
uint32_t cycle_count2 = 1;
uint32_t max_cycle_count2 = 0;

static void on_pwm_wrap() {
    // Clear the interrupt flag that brought us here
    pwm_clear_irq(SLICE_NUM);
    if(max_cycle_count1) {
        if(cycle_count1 == max_cycle_count1) {
            cycle_count1 = 1;
            SET_OUTPUT_PULSE_1
        } else {
            cycle_count1++;
        }
    }
    if(max_cycle_count2) {
        if(cycle_count2 == max_cycle_count2) {
            cycle_count2 = 1;
            SET_OUTPUT_PULSE_2
        } else {
            cycle_count2++;
        }
    }
}

static critical_section_t critical_section;

void start_pwm() {
    // Get some sensible defaults for the slice configuration. By default, the
    pwm_config config = pwm_get_default_config();
    // Mask slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(SLICE_NUM);
    pwm_set_irq_enabled(SLICE_NUM, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    // Set divider
    pwm_config_set_clkdiv(&config, 1.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(SLICE_NUM, &config, true);

    critical_section_init(&critical_section);

    // Set period
    uint16_t wrap_count = 124; // sysclock supposed at 125 MHz
    pwm_set_wrap(SLICE_NUM, wrap_count);
    // Set channel A output high before dropping
    pwm_set_chan_level(SLICE_NUM, PWM_CHAN_A, wrap_count/3);
    // Set the PWM running
    pwm_set_enabled(SLICE_NUM, true);
}

void core1_main() {
    uint32_t shadow_max_count;
    bool shadow_invert;    
    intercore_data_t inter_core_data;
    init_out_gpios();
    start_pwm();
    while (true) {
        queue_remove_blocking(&call_queue, &inter_core_data);
        shadow_invert = inter_core_data.invert1;
        shadow_max_count = inter_core_data.max_count1;
        critical_section_enter_blocking(&critical_section);
        if(shadow_invert != reverse1) {
            reverse1 = shadow_invert;
        }
        if(shadow_max_count != max_cycle_count1) {
            if(shadow_max_count < cycle_count1) {
                cycle_count1 = shadow_max_count;
            }
            max_cycle_count1 = shadow_max_count;
        }
        critical_section_exit(&critical_section);
        shadow_invert = inter_core_data.invert2;
        shadow_max_count = inter_core_data.max_count2;
        critical_section_enter_blocking(&critical_section);
        if(shadow_invert != reverse2) {
            reverse2 = shadow_invert;
        }
        if(shadow_max_count != max_cycle_count2) {
            if(shadow_max_count < cycle_count2) {
                cycle_count2 = shadow_max_count;
            }
            max_cycle_count2 = shadow_max_count;
        }
        critical_section_exit(&critical_section);
    }
}
