#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/memorymap.h>

/* SysTick interrupt handler */
void sys_tick_handler(void)
{
    gpio_toggle(GPIOC, GPIO13);
}

static void setup_systick(void)
{
    /* Setup SysTick to fire every 500ms */
    /* Assuming 72MHz system clock (default for STM32F103) */
    systick_set_reload(72000000 / 2 - 1); /* 500ms = 0.5Hz */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

int main(void)
{
    /* Enable GPIOC clock */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Set GPIO13 (PC13, LED pin) to 'output push-pull' */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    /* Setup SysTick timer for LED blinking */
    setup_systick();

    /* Main loop - now empty as LED is controlled by timer interrupt */
    while (1)
    {
        /* Could add other non-blocking tasks here */
    }

    return 0;
}
