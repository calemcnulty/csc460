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

volatile uint16_t sonar_timer;
volatile uint16_t sonar_elapsed;
volatile uint16_t mindist;
volatile uint8_t sentry_state;
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
	_delay_ms(5);
	PORTD &= ~_BV(PD6);
}

void sonar_update() {
		
		PORTB &= ~_BV(PB5);
		sonar_send();
		_delay_ms(100);
		if (sonar_elapsed == 0) PORTB |= _BV(PB5);
		
}

void search() {
		_delay_ms(10);
		Roomba_Drive(40, 200);
}

// find closest object within field of view of PIR sensor
void seek() {
	
	int i;
	
	Roomba_Drive(0, 0x8000);
	_delay_ms(10);		
	Roomba_PlaySong(sentry_state);
	_delay_ms(500);	
	
	// spin left
	Roomba_Drive(50, 1);
	
	// find closest object on left
	for (i = 0; i < 30; i++) {
		sonar_update();
		if (sonar_elapsed < mindist && sonar_elapsed > 10) mindist = sonar_elapsed;
	}
	
	// spin right
	Roomba_Drive(50, -1);
	
	// find closest object on right
	for (i = 0; i < 60; i++) {
		sonar_update();
		if (sonar_elapsed < mindist && sonar_elapsed > 10) mindist = sonar_elapsed;
	}
	
	// track left until intruder dead ahead
	Roomba_Drive(50, 1);
	while (sonar_elapsed > mindist) {
		sonar_update();
		_delay_ms(50);
	}
	Roomba_Drive(0, 0x8000);
	_delay_ms(50);
	sentry_state = LOCKED_ON;
}

void maintain_lock() {
	Roomba_PlaySong(sentry_state);
	_delay_ms(1500);
	
	// keep target more or less 1m away
	// 1m measured at 346 ticks back in the days of yore
	while (1) {
		_delay_ms(10);
		sonar_update();
		_delay_ms(50);
		if (sonar_elapsed > 370) {
			Roomba_Drive(40, 0x8000);
		} else if (sonar_elapsed < 320 && sonar_elapsed != 0) {
			Roomba_Drive(-40, 0x8000);
		} else {
			Roomba_Drive(0, 0x8000);
		}
	}
}

int main(void)
{
	PCICR |= (_BV(PCIE0) | _BV(PCIE2));  //enable interrupts 1 & 0
	PCMSK0 |= _BV(PCINT4);
	PCMSK2 |= _BV(PCINT23);

	//timer init
	TCNT1 = 0;
	
	//TODO fix the damned timer.
	/*OCR1A = 62500;
	TIMSK1 = _BV(OCIE1A);*/
	TCCR1B = _BV(CS12);
	
	UART_init();

	// initialize Data Direction Registers
	sei();
	DDRB |= _BV(PB5);
	DDRB &= ~_BV(PB4);
	DDRD |= _BV(PD6);
	DDRD &= ~_BV(PD7);
	DDRD &= ~_BV(PD0);
	DDRD |= _BV(PD1);
	DDRD |= _BV(PD3);

	// Set up Roomba
	uint8_t notes[3][8] = {{60, 69, 60, 69, 60, 69, 60, 69},
						   {64, 73, 64, 73, 64, 73, 64, 73},
						   {69, 78, 87, 78, 69, 78, 87, 78}};
	uint8_t durrs[8] = {16, 16, 16, 16, 16, 16, 16, 16};
	Roomba_Init();
	_delay_ms(100);
	Roomba_LoadSong(SEARCHING, notes[SEARCHING], durrs, 8);
	Roomba_LoadSong(SEEKING, notes[SEEKING], durrs, 8);
	Roomba_LoadSong(LOCKED_ON, notes[LOCKED_ON], durrs, 8);
	_delay_ms(100);
	
	Roomba_PlaySong(0);
	
	// PIR warmup
	_delay_ms(15000);
	sentry_state = SEARCHING;
	s = (char *)malloc(16);
    while(1) {
		/*sonar_update();
		s = itoa(sonar_elapsed, s, 10);
		strcat(s, "\n");
		UART_send(s);
		*/
		switch (sentry_state) {
			case SEARCHING:
				search();
				break;
			case SEEKING:
				sonar_update();
				if (sonar_elapsed > 10) mindist = sonar_elapsed;
				seek();
				break;
			case LOCKED_ON:
				sonar_update();
				maintain_lock();
				break;
			default:
				sonar_update();	
		}
    }			
}

// PIR interrupt handler
ISR(PCINT0_vect) {

    if (PIR1_ON) {
		if (sentry_state == SEARCHING) {			
			sentry_state = SEEKING;		
		}
		
	}
}


// TODO: on receiving signal, set start, wait for square wave to drop, measure elapsed, then send it.
ISR(PCINT2_vect) {
	
	if (PIND & _BV(PD7)) {
		sonar_timer = TCNT1;
	} else {
		sonar_elapsed = TCNT1 - sonar_timer;
	}
}