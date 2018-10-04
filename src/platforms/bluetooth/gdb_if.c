#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include "gdb_if.h"
#include "general.h"

#define FIFO_SIZE 128

/* RX Fifo buffer */
static volatile uint8_t buf_rx[FIFO_SIZE];
/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static volatile uint8_t buf_rx_in;
/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static volatile uint8_t buf_rx_out;

int gdb_if_init(void)
{
    // Initialize GDB RSP UART
    rcc_periph_clock_enable(GDBUSART_CLK);

    GDBUART_PIN_SETUP();

    /* Setup UART parameters. */
    usart_set_baudrate(GDBUSART, GDBUSART_BAUDRATE);
    usart_set_databits(GDBUSART, 8);
    usart_set_stopbits(GDBUSART, USART_STOPBITS_1);
    usart_set_mode(GDBUSART, USART_MODE_TX_RX);
    usart_set_parity(GDBUSART, USART_PARITY_NONE);
    usart_set_flow_control(GDBUSART, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(GDBUSART);

    /* Enable interrupts */
    GDBUSART_CR1 |= USART_CR1_RXNEIE;
    nvic_set_priority(GDBUSART_IRQ, IRQ_PRI_GDBUSART);
    nvic_enable_irq(GDBUSART_IRQ);
    return 0;
}

/*
 * Read a character from the UART RX and stuff it in a software FIFO.
 * Allowed to read from FIFO out pointer, but not write to it.
 * Allowed to write to FIFO in pointer.
 */
void GDBUSART_ISR(void)
{
    uint32_t err = USART_SR(GDBUSART);
    char c = usart_recv(GDBUSART);
    if (err & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
        return;

    /* Turn on LED */
    gpio_set(LED_PORT_UART, LED_UART);

    /* If the next increment of rx_in would put it at the same point
    * as rx_out, the FIFO is considered full.
    */
    if (((buf_rx_in + 1) % FIFO_SIZE) != buf_rx_out)
    {
        /* insert into FIFO */
        buf_rx[buf_rx_in++] = c;

        /* wrap out pointer */
        if (buf_rx_in >= FIFO_SIZE)
        {
            buf_rx_in = 0;
        }
    }
}

unsigned char gdb_if_getchar(void)
{
    /* copy from uart FIFO into local usb packet buffer */
    while (buf_rx_in == buf_rx_out);
    
    unsigned char ret = buf_rx[buf_rx_out++];
    if (buf_rx_out >= FIFO_SIZE)
    {
        buf_rx_out = 0;
    }
    return ret;
}

unsigned char gdb_if_getchar_to(int timeout)
{
    platform_timeout t;
    platform_timeout_set(&t, timeout);
    
    while (!platform_timeout_is_expired(&t) && (buf_rx_in == buf_rx_out));

    if (buf_rx_in != buf_rx_out)
    {
        unsigned char ret = buf_rx[buf_rx_out++];
        if (buf_rx_out >= FIFO_SIZE)
        {
            buf_rx_out = 0;
        }
        return ret;
    }
    return -1;
}

void gdb_if_putchar(unsigned char c, int flush)
{
    (void)flush;
    usart_send_blocking(GDBUSART, c);
    return;
}
