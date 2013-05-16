/*
 * GccApplication1.c
 *
 * Created: 5/7/2013 2:55:42 PM
 *  Author: nrqm
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int main(void)
{
	sei();
	DDRB = _BV(PB5);
	
	TCNT1 = 0;
	OCR1A = 62500;
	TIMSK1 = _BV(OCIE1A);
	TCCR1B = _BV(CS12);
	
    while(1)
    {
    }
}

ISR(TIMER1_COMPA_vect)
{
	PORTB ^= _BV(PB5);
	OCR1A += 62500;
}