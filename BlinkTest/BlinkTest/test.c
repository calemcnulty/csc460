/*
 * test.c
 *
 *  Created on: 4-Feb-2009
 *      Author: nrqm
 */

#include "roomba/roomba.h"
#include "roomba/roomba_sci.h"
#include "uart/uart.h"
#include <util/delay.h>
//#include "awesomedelay.h"
#include "avr/interrupt.h"
#include "radio/radio.h"

#define     clock8MHz()    CLKPR = _BV(CLKPCE); CLKPR = 0x00;

uint8_t isStation1()
{
	// the station is the transmitter if PINC7 is connected to ground (assumes that DDRC7 is set to input and that
	// the receiver has it connected to 3.3V or the pull-up resistor is enabled).
	return (PINC & _BV(PC7)) == 0;
}

uint8_t station1_addr[5] = { 0xED, 0xB7, 0xED, 0xB7, 0xED };
uint8_t station2_addr[5] = { 0xED, 0xB7, 0xED, 0xB7, 0xEE };

volatile uint8_t rxflag = 0;

radiopacket_t packet;

void roomba_Drive( int16_t velocity, int16_t radius )
{
	uart_putchar(DRIVE);
	uart_putchar(HIGH_BYTE(velocity));
	uart_putchar(LOW_BYTE(velocity));
	uart_putchar(HIGH_BYTE(radius));
	uart_putchar(LOW_BYTE(radius));
}

int main()
{
	uint8_t i;
	clock8MHz();

	cli();

	DDRC = 0x00;
	PORTC = 0xFF;
	DDRD = 0xFF;
	PORTD = 0x00;

	Roomba_Init();

	Radio_Init();

	if (isStation1())
	{
		Radio_Configure_Rx(RADIO_PIPE_0, station1_addr, ENABLE);
	}
	else
	{
		Radio_Configure_Rx(RADIO_PIPE_0, station2_addr, ENABLE);
	}
	Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);

	sei();

	/* UART test - drive straight forward at 100 mm/s for 1 second
	roomba_Drive(100, 0x8000);

	_delay_1s();

	roomba_Drive(0, 0);

	for (;;);*/

	for (;;)
	{
		if (rxflag)
		{
			_delay_ms(20);
			// Copy the received packet into the radio packet structure.  If there are no more packets,
			// then clear the rxflag so that the interrupt will set it next time a packet is received.
			if (Radio_Receive(&packet) != RADIO_RX_MORE_PACKETS)
			{
				rxflag = 0;
			}

			// If the packet is not a command, blink an error and don't do anything.
			if (packet.type != COMMAND)
			{
				PORTD ^= _BV(PD7);
				continue;
			}

			if (packet.payload.command.command == START ||
					packet.payload.command.command == BAUD ||
					packet.payload.command.command == CONTROL ||
					packet.payload.command.command == SAFE ||
					packet.payload.command.command == FULL ||
					packet.payload.command.command == SENSORS)
			{
				// Don't pass the listed commands to the Roomba.
				continue;
			}

			// Output the command to the Roomba, followed by its arguments.
			uart_putchar(packet.payload.command.command);
			for (i = 0; i < packet.payload.command.num_arg_bytes; i++)
			{
				uart_putchar(packet.payload.command.arguments[i]);
			}

			// Set the radio's destination address to be the remote station's address
			Radio_Set_Tx_Addr(packet.payload.command.sender_address);

			// Update the Roomba sensors into the packet structure that will be transmitted.
			Roomba_UpdateSensorPacket(1, &packet.payload.sensors.sensors);
			Roomba_UpdateSensorPacket(2, &packet.payload.sensors.sensors);
			Roomba_UpdateSensorPacket(3, &packet.payload.sensors.sensors);

			// send the sensor packet back to the remote station.
			packet.type = SENSOR_DATA;

			if (Radio_Transmit(&packet, RADIO_WAIT_FOR_TX) == RADIO_TX_MAX_RT)
			{
				PORTD ^= _BV(PD4);	// flash red if the packet was dropped
			}
			else
			{
				PORTD ^= _BV(PD5);	// flash green if the packet was received correctly
			}
		}
	}

	return 0;
}

void radio_rxhandler(uint8_t pipenumber)
{
	rxflag = 1;
	PORTD ^= _BV(PD7);
}
