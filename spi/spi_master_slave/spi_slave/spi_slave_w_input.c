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

#define SPI0_RESET_BITFIELD (0b1 << 16)
#define SPI0_RESET          (*(volatile uint32_t*)RESETS_BASE)

#define BSY_MASK  0b10000UL
#define RFF_MASK  0b1000UL
#define RNE_MASK  0b100UL
#define TNF_MASK  0b10UL
#define TFE_MASK  0b1UL

#define ENDSTDIN	255
#define CR		13

void flushTxFifo(void)
{
    // Toggle reset of SPI0 to flush Tx FIFO
    SPI0_RESET |= SPI0_RESET_BITFIELD;
    SPI0_RESET &= ~SPI0_RESET_BITFIELD;

    // Bare min?
    spi_init(spi_default, 1000 * 100000);
    spi_set_slave(spi_default, true);
}

int main() {
    // Enable UART so we can print
    stdio_init_all();
#if !defined(spi_default) || !defined(PICO_DEFAULT_SPI_SCK_PIN) || !defined(PICO_DEFAULT_SPI_TX_PIN) || !defined(PICO_DEFAULT_SPI_RX_PIN) || !defined(PICO_DEFAULT_SPI_CSN_PIN)
#warning spi/spi_slave example requires a board with SPI pins
    puts("Default SPI pins were not defined");
#else

    // For usb-minicom
    sleep_ms(5000);

    // LED pin for debug purposes
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    for(uint8_t i = 0 ; i < 3 ; i++)
    {
        gpio_put(LED_PIN, 1);
        printf("pico is alive!\n");
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }

    const uint READY_PIN = 20; // GP20
    gpio_init(READY_PIN);
    gpio_set_dir(READY_PIN, GPIO_OUT);
    gpio_put(READY_PIN, 0); // This should be the default (Pull-down enabled)

    // Enable SPI 0 at 1 MHz and connect to GPIOs
    spi_init(spi_default, 1000 * 100000);
    spi_set_slave(spi_default, true);
    gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI);
    // Make the SPI pins available to picotool
    bi_decl(bi_4pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, PICO_DEFAULT_SPI_CSN_PIN, GPIO_FUNC_SPI));

    // Pointers for some basic SPI reg access
    volatile uint32_t* pSSPSR = (volatile uint32_t*)(SPI0_BASE + 0xc);
    volatile uint32_t* pSSPDR = (volatile uint32_t*)(SPI0_BASE + 0x8);
    volatile uint32_t* pSSPCR0 = (volatile uint32_t*)(SPI0_BASE + 0x0);
    volatile uint32_t* pSSPCR1 = (volatile uint32_t*)(SPI0_BASE + 0x4);

    // Change Data Size Select to 8 bits
    *pSSPCR0 &= ~(0xf << 0);
    *pSSPCR0 |= 0b0111 << 0;

    // Test and status prints 
    printf("SPI slave example w/ user input:\n");
    printf("SPI0_BASE: 0x%08lx\n", SPI0_BASE);
    printf("SSPCR0:    0x%08lx\n", *pSSPCR0);
    printf("SSPCR1:    0x%08lx\n", (*(volatile uint32_t*)(SPI0_BASE + 0x4)));
    printf("SSPCPSR:   0x%08lx\n", (*(volatile uint32_t*)(SPI0_BASE + 0x10)));
    printf("SSPSR:     0x%08lx\n\n", *pSSPSR);
    printf("Enter new SPI message: ");
    fflush(stdout);

    // Index var for SPI message trasmit
    uint8_t i = 0;
    // Variables for grabbing new SPI message
    char message[32] = "Hello, friend.\n";
    for(uint16_t i = 0 ; i < 8 ; i++)
    {
        *pSSPDR = (uint8_t)message[i++];
    }
    uint8_t j = 0;
    char chr;
    while(1)
    {
        // Make sure ready pin is cleared
        gpio_put(READY_PIN, 0);

        // Attempt to grab new character if preset
        chr = getchar_timeout_us(0);
        while (chr != ENDSTDIN)
        {
            message[j++] = chr;
            printf("%c", chr);
            fflush(stdout);
            if (chr == CR || j == sizeof(message) - 1)
            {
                // Full message has occurred
                message[j - 1] = 0xA; // Swap CR for LF
                message[j] = 0; // Term char
                for(uint8_t i = j + 1 ; i < 31 ; i++)
                {
                    message[i] = 0; // Make sure the rest of the characters are 0;
                }
                j = 0; // Reset index
                // printf("\n\t%s\t\n", message);
                // for(uint16_t i = 0 ; i < 32 ; i++)
                // {
                //     printf("%2x ", message[i]);
                // }
                printf("\nEnter new SPI message: ");
                fflush(stdout);

                // Reset SPI0 and reenable as slave to flush Tx FIFO
                flushTxFifo();

                // Fill TX FIFO, overwritting previous data
                i = 0;
                *pSSPDR = (uint8_t)message[i++];

                // Set READY pin
                gpio_put(READY_PIN, 1);

                break; 
            }
            chr = getchar_timeout_us(0);
        }

        if((*pSSPSR & TNF_MASK))
        {
            *pSSPDR = (uint8_t)message[i++];
            if(!message[i - 1])
            {
                i = 0;
            }
        }

        // if((*pSSPSR & RNE_MASK))
        // {
        //     printf("%c", (uint8_t)*pSSPDR);
        // }
    }
#endif
}
