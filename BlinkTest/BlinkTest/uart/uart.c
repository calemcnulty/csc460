/**
 * @file   uart.c
 * @author Justin Tanner
 * @date   Sat Nov 22 21:32:03 2008
 *
 * @brief  UART Driver targetted for the AT90USB1287
 *
 */
#include "uart.h"

#ifndef F_CPU
#warning "F_CPU not defined for uart.c."
#define F_CPU 11059200UL
#endif

static volatile uint8_t uart_buffer[UART_BUFFER_SIZE];
static volatile uint8_t uart_buffer_index;

/**
 * Initalize UART
 *
 */
void uart_init(UART_BPS bitrate)
{
	UCSR1A = _BV(U2X1);
	UCSR1B = _BV(RXEN1) | _BV(TXEN1) | _BV(RXCIE1);
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);

	UBRR1H = 0;	// for any speed >= 9600 bps, the UBBR value fits in the low byte.

	// See the appropriate AVR hardware specification for a table of UBBR values at different
	// clock speeds.
	switch (bitrate)
	{
#if F_CPU==8000000UL
	case UART_19200:
		UBRR1L = 51;
		break;
	case UART_38400:
		UBRR1L = 25;
		break;
	case UART_57600:
		UBRR1L = 16;
		break;
	default:
		UBRR1L = 51;
#elif F_CPU==16000000UL
	case UART_19200:
		UBRR1L = 103
		break;
	case UART_38400:
		UBRR1L = 51;
		break;
	case UART_57600:
		UBRR1L = 34;
		break;
	default:
		UBRR1L = 103;
#elif F_CPU==18432000UL
	case UART_19200:
		UBRR1L = 119;
		break;
	case UART_38400:
		UBRR1L = 59;
		break;
	case UART_57600:
		UBRR1L = 39;
		break;
	default:
		UBRR1L = 119;
		break;
#else
#warning "F_CPU undefined or not supported in uart.c."
	default:
		UBRR1L = 71;
		break;
#endif
	}

    uart_buffer_index = 0;
}

/**
 * Transmit one byte
 * NOTE: This function uses busy waiting
 *
 * @param byte data to trasmit
 */
void uart_putchar(uint8_t byte)
{
    /* wait for empty transmit buffer */
    while (!( UCSR1A & (1 << UDRE1)));

    /* Put data into buffer, sends the data */
    UDR1 = byte;
}

/**
 * Receive a single byte from the receive buffer
 *
 * @param index
 *
 * @return
 */
uint8_t uart_get_byte(int index)
{
    if (index < UART_BUFFER_SIZE)
    {
        return uart_buffer[index];
    }
    return 0;
}

/**
 * Get the number of bytes received on UART
 *
 * @return number of bytes received on UART
 */
uint8_t uart_bytes_received(void)
{
    return uart_buffer_index;
}

/**
 * Prepares UART to receive another payload
 *
 */
void uart_reset_receive(void)
{
    uart_buffer_index = 0;
}

/**
 * UART receive byte ISR
 */
ISR(USART1_RX_vect)
{
    uart_buffer[uart_buffer_index] = UDR1;
    uart_buffer_index = (uart_buffer_index + 1) % UART_BUFFER_SIZE;
}





