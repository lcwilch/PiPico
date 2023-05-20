// Copyright (c) 2021 Michael Stoops. All rights reserved.
// Portions copyright (c) 2021 Raspberry Pi (Trading) Ltd.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
//    disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
//    following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
//    products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// SPDX-License-Identifier: BSD-3-Clause
//
// Example of an SPI bus slave using the PL022 SPI interface

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

#define SPI0_BASE       0x4003c000UL
#define BUF_LEN         8

int main() {
    // Enable UART so we can print
    stdio_init_all();
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_slave example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    // For usb-minicom
    sleep_ms(5000);

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("SPI slave example\n");

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 100000);
    spi_set_slave(spi_default, true);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    uint16_t in_buf[BUF_LEN];

    for (uint32_t i = 0; i < 5; i++)
    {
        gpio_put(LED_PIN, 1);
        printf("pico is alive!\n");
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    volatile uint32_t* pSSPSR = SPI0_BASE + 0xc;
    volatile uint32_t* pSSPDR = SPI0_BASE + 0x8;
    volatile uint32_t* pSSPCR0 = SPI0_BASE + 0x0;
    const uint8_t BSY_MASK = 0b10000;
    const uint8_t RFF_MASK = 0b1000;
    const uint8_t RNE_MASK = 0b100;
    const uint8_t TNF_MASK = 0b10;
    const uint8_t TFE_MASK = 0b1;


    printf("SSPCR0:  0x%08lx\n", (*(volatile uint32_t*)(SPI0_BASE + 0x0)));
    printf("SSPCR1:  0x%08lx\n", (*(volatile uint32_t*)(SPI0_BASE + 0x4)));
    printf("SSPCPSR: 0x%08lx\n", (*(volatile uint32_t*)(SPI0_BASE + 0x10)));
    printf("SSPSR:   0x%08lx\n", *pSSPSR);

    printf("waiting for SPI write from master...\n");

    char message[] = "Hello, friend.\n";

    uint8_t i = 0;

    // Change Data Size Select to 8 bits
    *pSSPCR0 &= ~(0xf << 0);
    *pSSPCR0 |= 0b0111 << 0;

    while(1)
    {

        if((*pSSPSR & TFE_MASK))
        {
            if(message[i])
            {
                *pSSPDR = (uint8_t)message[i++];
            }
            else
            {
                i = 0;
            }
        }
        if((*pSSPSR & RNE_MASK))
        {
            printf("%c", (uint8_t)*pSSPDR);
        }
    }
#endif
}
