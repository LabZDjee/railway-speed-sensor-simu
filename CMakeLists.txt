add_executable(speed_sensor
 float_equality_ulp.c
 out-gpios.c
 pwm-managed.c
 speed-sensor.c
 speed-sensor-util.c
)

# pull in common dependencies
target_link_libraries(speed_sensor pico_stdlib pico_multicore hardware_pwm)

# enable usb output, disable uart output
pico_enable_stdio_usb(speed_sensor 1)
pico_enable_stdio_uart(speed_sensor 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(speed_sensor)

# add url via pico_set_program_url
example_auto_set_url(speed_sensor)
