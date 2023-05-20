/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include <stdio.h>

int main() {
#ifndef PICO_DEFAULT_LED_PIN
#warning blink example requires a board with a regular LED
#else
    stdio_init_all();
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        printf("Hello, world!\n");
        gpio_put(LED_PIN, 1);
        sleep_ms(1000);
        printf("Goodbye, world...\n");
        gpio_put(LED_PIN, 0);
        sleep_ms(1000);
    }
#endif
}
