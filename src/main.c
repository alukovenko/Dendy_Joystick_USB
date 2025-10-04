#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define DELAY_COUNT 500000

static void delay(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        __asm__("nop");
    }
}

int main(void)
{
    /* Enable GPIOC clock */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Set GPIO13 (PC13, LED pin) to 'output push-pull' */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    while (1)
    {
        gpio_toggle(GPIOC, GPIO13); // Toggle LED state
        delay(DELAY_COUNT);         // Delay for ~500 ms
    }

    return 0;
}
