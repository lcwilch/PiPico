add_executable(i2c_slave
                i2c_slave.c
        )

# Pull in basic dependencies
target_link_libraries(i2c_slave pico_stdlib hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(i2c_slave 1)
pico_enable_stdio_uart(i2c_slave 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(i2c_slave)

# add url via pico_set_program_url
example_auto_set_url(i2c_slave)
