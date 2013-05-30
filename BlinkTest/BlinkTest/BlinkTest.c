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
#include "roomba.h"

#define BAUDRATE 19200
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define SONAR_METER 364

// bit masks for flags
#define PIR1_ON		PINB & _BV(PB4)
#define PIR2_ON		PINB & _BV(PB3)
#define SEARCHING	0
#define SEEKING		1
#define LOCKED_ON	2

volatile uint16_t timer;
volatile uint16_t elapsed;
volatile uint16_t mindist;
volatile uint8_t sentry_state = 0;
char* s;
roomba_sensor_data_t sensor;

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

void sonar_send() {
	PORTD |= _BV(PD6);
	_delay_ms(100);
	PORTD &= ~_BV(PD6);
}

void sonar_update() {

	if (elapsed) {
		elapsed = 0;
		_delay_ms(100);
		sonar_send();
	}
}

void seek() {
	int time_itter = TCNT1;
	Roomba_Drive(200, 20);
	while (time_itter - TCNT1 > 500) {
		if (mindist > elapsed + 10) {
			mindist = elapsed;
			sonar_update();
		}
	}
	Roomba_Drive(-200, 20);
	time_itter = TCNT1;
	while (time_itter - TCNT1){
		
	}
}


int main(void)
{
	PCICR |= (_BV(PCIE0) | _BV(PCIE2));  //enable interrupts 1 & 0
	PCMSK0 |= _BV(PCINT4);
	PCMSK2 |= _BV(PCINT23);

	UART_init();
	s = (char*)malloc(sizeof(char));

	sei();
	DDRB |= _BV(PB5);
	DDRB &= ~_BV(PB4);
	DDRB &= ~_BV(PB3);
	DDRD |= _BV(PD6);
	DDRD &= ~_BV(PD7);
	TCCR1B = _BV(CS12);
	DDRD &= ~_BV(PD0);
	DDRD |= _BV(PD1);
	DDRD |= _BV(PD3);

	uint8_t notes[8] = {60, 69, 60, 69, 60, 69, 60, 69};
	uint8_t durrs[8] = {16, 16, 16, 16, 16, 16, 16, 16};
	

	/*
	Roomba_Init();
	_delay_ms(100);
	Roomba_LoadSong(0, notes, durrs, 8);
	_delay_ms(100);
	Roomba_PlaySong(0);
	_delay_ms(100);
	
	Roomba_Workout();*/

	elapsed = 1; //ugly hack to force conditional on first iteration

    while(1) {
		if (elapsed) {
			s = itoa(elapsed, s, 10);
			strcat(s, "\n");
			UART_send(s);
			sonar_update();
		}
		/*switch (sentry_state) {
			case SEARCHING:
				sonar_update();
				break;
			case SEEKING:
				seek();
				break;
			case LOCKED_ON:
				//lock_maintain();
				break;
			default:
				sonar_update();	
		}	*/
    }			
}

// PIR interrupt handler
ISR(PCINT0_vect) {
	
	if (PIR1_ON) {
		if (sentry_state == SEARCHING) sentry_state = SEEKING;
		mindist = elapsed;
	} else {
	}
	if (PIR2_ON) {
		if (sentry_state == SEARCHING) sentry_state = SEEKING;
		mindist = elapsed;
	} else {
	}
}


// TODO: on receiving signal, set start, wait for square wave to drop, measure elapsed, then send it.
ISR(PCINT2_vect) {
	
	if (PIND & _BV(PD7)) {
		timer = TCNT1;
		elapsed = 0;
	} else {
		elapsed = TCNT1 - timer;
	}
}
