//##########################################################
//##                      ARMS MODULE CONTROL             ##
//## CM-700 (Atmega2561) 							      ##
//##                                           2011.23.11 ##
//##########################################################

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "dynamixel.h"
#include "serial.h"
#include "cmdATM.c"
#include "cmdATM.h"
#include <util/delay.h>
//#include "Control.c"
//#include "kinematics.c"


int count = 0;

void Leds();

int main(void)
{
	//int Value = 0;
	
	DDRC  = 0x7F;
	PORTC = 0x7D;
	serial_initialize(57600);				// USART Initialize
	//serial_initialize(250000);				// USART Initialize
	dxl_initialize( 0, 7 ); // Not using device index
	sei();
	SetUpServos();
	SetConstants();
	SetKConstants();

	printf("ARMS CONTROL\r" );

	while (1)
	{
	
		
	if(cmdReady)
	{
		 if(!execCommand()) printf("Unknown command\r");
	}
	
	
	if(doLoop)
	{
		executeControl();
		count = count + 100;
	}
	
	if(doGripper)
	{
		executeGripper();
	}
		
		_delay_ms(3);
	

	Leds();

	}

	return 1;
}


void Leds()
{

if(doLoop) PORTC &= 0b0111111;
	else PORTC |= 0b1000000;

if(doReading) PORTC &= 0b1011111;
	else PORTC |= 0b0100000;

if(doControl) PORTC &= 0b1101111;
	else PORTC |= 0b0010000;

if(count>5000)
	{
		PORTC ^= 0b0000010;
		count = 0;
	}
	count++;
}

