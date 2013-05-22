/*
 * GccApplication1.c
 *
 * Created: 5/7/2013 2:55:42 PM
 *  Author: nrqm
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE 57600
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)

volatile uint16_t timer;
volatile uint16_t elapsed;
char* s;

void UART_init(void) {
	
	UBRR0H = (uint8_t)(BAUD_PRESCALER >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
}

void UART_send(char* data) {
	
	while (*data) {
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *data;
		data++;
	}
}

unsigned char UART_receive(void) {
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void sonar_send () {
	PORTD |= _BV(PD6);
	_delay_ms(100);
	PORTD &= ~_BV(PD6);
}

int main(void)
{
	s = (char *)malloc(sizeof(char) * 16);

	PCICR |= (_BV(PCIE0) | _BV(PCIE2));  //enable interrupts 1 & 0
	PCMSK0 |= _BV(PCINT4);
	PCMSK2 |= _BV(PCINT23);
	
	UART_init();

	sei();
	DDRB |= _BV(PB5);
	DDRB &= ~_BV(PB4);
	DDRD |= _BV(PD6);
	DDRD &= ~_BV(PD7);
	TCCR1B = _BV(CS12);

	elapsed = 1; //ugly hack to force conditional on first iteration

    while(1) {	
		if (elapsed) {
			itoa(elapsed, s, 10);
			strcat(s, "\n");
			UART_send(s);
			elapsed = 0;	
			_delay_ms(100);
			sonar_send();
		}
    }
}

ISR(PCINT0_vect) {
	
	if (PINB & _BV(PB4)) {
		PORTB |= _BV(PB5);
	} else {
		PORTB &= ~_BV(PB5);
	}
}


// TODO: on receiving signal, set start, wait for square wave to drop, measure elapsed, then send it.
ISR(PCINT2_vect) {
	
	if (PIND & _BV(PD7)) {
		timer = TCNT1;
		elapsed = 0;
		PORTB |= _BV(PB5);	
	} else {
		elapsed = TCNT1 - timer;
		PORTB &= ~_BV(PB5);
	}
}


