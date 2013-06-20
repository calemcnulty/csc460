/*
 * GccApplication1.c
 *
 * Created: 5/7/2013 2:55:42 PM
 *  Author: Cale McNulty
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "roomba.h"

#define BAUDRATE 19200
#define BAUD_PRESCALER (((F_CPU / (BAUDRATE * 16UL))) - 1)
#define SONAR_METER_LOW 10500
#define SONAR_METER_HIGH 15000

// bit masks for flags
#define PIR1_ON		PIND & _BV(PD7)
#define SEARCHING	0
#define SEEKING		1
#define LOCKED_ON	2

volatile uint16_t sonar_start;
volatile uint16_t sonar_end;
volatile uint16_t sonar_elapsed;
volatile uint16_t mindist;
volatile uint8_t sentry_state;
char* s;
roomba_sensor_data_t sensor;

// initialize UART registers
void UART_init(void) {
	
	UBRR0H = (uint8_t)(BAUD_PRESCALER >> 8);
	UBRR0L = (uint8_t)(BAUD_PRESCALER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
}

// send a bytestream over UART
void UART_send(char* data) {
	
	while (*data) {
		while(!(UCSR0A & (1<<UDRE0)));
		UDR0 = *data;
		data++;
	}
}

// receive data from UART
unsigned char UART_receive(void) {
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void sonar_init() {
	
	// set up ICP register
	DDRB &= ~_BV(PB0);
	PORTB |= _BV(PB0);
	
	// set timer control reg to trigger on rising
	TCCR1B |= _BV(ICES1);
	
	// enable noise filter
	TCCR1B |= _BV(ICNC1);
	
	// enable input capture for timer mask
	TIMSK1 |= _BV(ICIE1);
	
	// set up prescaler
	TCCR1B |=  _BV(CS11);
	
}

// send a trigger pulse then clear the timer counter
void sonar_send() {
	
	PORTD |= _BV(PD6);
	_delay_us(150);
	PORTD &= ~_BV(PD6);
	
	// easier than dealing with overflows
	TCNT1 = 0;
	
}

// send a timer pulse then wait long enough for it to get back
void sonar_update() {
	
	sonar_send();
	_delay_ms(100);

}

// drive around in a circle.  PIR interrupt 
// will break out of this state
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
	_delay_ms(50);	
	
	// spin left
	Roomba_Drive(50, 1);
	
	// find closest object on left
	for (i = 0; i < 30; i++) {
		sonar_update();
		if (sonar_elapsed < mindist) mindist = sonar_elapsed;
	}
	
	// spin right
	Roomba_Drive(50, -1);
	
	// find closest object on right
	for (i = 0; i < 60; i++) {
		sonar_update();
		if (sonar_elapsed < mindist) mindist = sonar_elapsed;
	}
	
	// track left until intruder dead ahead
	Roomba_Drive(50, 1);
	while (sonar_elapsed > mindist + 100) {
		sonar_update();
	}
	Roomba_Drive(0, 0x8000);
	_delay_ms(50);
	sentry_state = LOCKED_ON;
}

void maintain_lock() {
	Roomba_PlaySong(sentry_state);
	_delay_ms(15);
	
	// keep target more or less 1m away
	while (1) {
		sonar_update();
		
		if (sonar_elapsed > SONAR_METER_HIGH) {
			Roomba_Drive(40, 0x8000);
		} else if (sonar_elapsed < SONAR_METER_LOW) {
			Roomba_Drive(-40, 0x8000);
		} else {
			Roomba_Drive(0, 0x8000);
		}
	}
}

void sentry_init() {

	//enable interrupts
	PCICR |= (_BV(PCIE0) | _BV(PCIE2));  
	PCMSK2 |= _BV(PCINT23);
	
	UART_init();
	sonar_init();
	sei();
	
	// initialize Data Direction Registers
	DDRB |= _BV(PB5);
	DDRB &= ~_BV(PB4);
	DDRB &= ~_BV(PD7);
	DDRD |= _BV(PD6);
	DDRD &= ~_BV(PD0);
	DDRD |= _BV(PD1);
	DDRD |= _BV(PD3);
}

// initialize Roomba, load songs, and the like
void Roomba_Setup () {

	// each state has a progressively higher-pitched song
	uint8_t notes[3][8] = {{60, 69, 60, 69, 60, 69, 60, 69},
						   {64, 73, 64, 73, 64, 73, 64, 73},
						   {69, 78, 87, 78, 69, 78, 87, 78}};
	uint8_t durrs[8] = {16, 16, 16, 16, 16, 16, 16, 16};
	
	Roomba_Init();
	_delay_ms(10);
	Roomba_LoadSong(SEARCHING, notes[SEARCHING], durrs, 8);
	Roomba_LoadSong(SEEKING, notes[SEEKING], durrs, 8);
	Roomba_LoadSong(LOCKED_ON, notes[LOCKED_ON], durrs, 8);
	_delay_ms(10);	
}

// my kingdom for a printf
void debug() {
	
	s = (char*)malloc(64);
	while(1) {
		sonar_update();
		sprintf(s, "%u, %u, %u\n", sonar_start, sonar_end, sonar_elapsed);
		UART_send(s);
	}	
}

int main(void)
{
	sentry_init();
	Roomba_Setup();
	
	// verify Roomba is working with a nice ditty	
	Roomba_PlaySong(0);
	
	// PIR warmup
	_delay_ms(15000);
	sentry_state = SEARCHING;

	while (1) {
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
ISR(PCINT2_vect) {

    if (PIR1_ON) {
		if (sentry_state == SEARCHING) {			
			sentry_state = SEEKING;		
		}		
	}
}

// ICP timer interrupt handler for sonar
ISR(TIMER1_CAPT_vect) {
	
	// if echo is a rising edge	
	if (TCCR1B & _BV(ICES1)) {
	
		// set initial time to Input Capture Register
		sonar_start = ICR1;
		
		// set Timer Control to trigger on falling edge
		TCCR1B &= ~_BV(ICES1);
	
	// if falling edge	
	} else { //if (TCCR1B ^ _BV(ICES1)) {

		sonar_end = ICR1;
		
		// calculate elapsed
		sonar_elapsed = sonar_end - sonar_start;
		
		//tell timer control register to look for rising edge
		TCCR1B |= _BV(ICES1);
	}
}
