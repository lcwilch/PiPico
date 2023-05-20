#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/structs/i2c.h"

int main() {
    // Enable UART so we can print status output
    stdio_init_all();
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c0, 100 * 1000);
    i2c_set_slave_mode(i2c0, true, 0x55);
    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);
    gpio_pull_up(16);
    gpio_pull_up(17);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(16, 17, GPIO_FUNC_I2C));

    sleep_ms(3000);
    printf("I2C Slave test:\n", i2c0_hw->sar);
    printf("saddr: 0x%x\n", i2c0_hw->sar);
    printf("con: 0x%08x\n", i2c0_hw->con);
    printf("enable: 0x%x\n", i2c0_hw->enable);
    uint8_t data;

    // i2c_write_blocking(i2c0, 0x44, &data, 1, false);
    
    while (1)
    {
        // if(i2c_get_read_available(i2c0))
        // {
        //     i2c_read_raw_blocking(i2c0, &data, 1);
        //     printf("%c", data);
        // }

        // Check if Recieve FIFO not empty
        if (i2c0_hw->status & (I2C_IC_STATUS_RFNE_BITS))
        {
            // Grab data
            data = (uint8_t)i2c0_hw->data_cmd;
            printf("%c", data);
        }
    }

    return 0;
#endif
}
