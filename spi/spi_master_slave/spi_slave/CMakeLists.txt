add_executable(spi_slave
        spi_slave_w_input.c
        )

# Pull in basic dependencies
target_link_libraries(spi_slave pico_stdlib hardware_spi)

# enable usb output, disable uart output
pico_enable_stdio_usb(spi_slave 1)
pico_enable_stdio_uart(spi_slave 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_slave)

# add url via pico_set_program_url
example_auto_set_url(spi_slave)
