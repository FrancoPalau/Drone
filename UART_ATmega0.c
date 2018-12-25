/*
 * UART_ATmega0.c
 *
 * Created: 3/10/2017 1:25:49 a. m.
 *  Author: Tincho
 */ 

#ifndef F_CPU
	#define F_CPU 16000000
#endif
//#include "UART_ATmega.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>

uint8_t mi_UART_Init0(uint16_t brate,uint8_t Txinterr,uint8_t Rxinterr)
{
	UBRR0 = F_CPU/16/brate-1;				// Configura baudrate. Ver en sección UART de datasheet
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);			// Habilita bits TXEN0 y RXEN0
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);		// USBS0=1 2 bits stop, UCSZxx=3 8 bits
	if (Txinterr)
	UCSR0B|= (1<<TXCIE0);	// Interrupcion Tx UART0
	if (Rxinterr)
	UCSR0B|= (1<<RXCIE0);	// Interrupcion Rx UART0
	return 1;
}

int mi_putc0(char c, FILE *stream)
{
	while(!(UCSR0A & (1<<UDRE0)) ); // Espera mientras el bit UDRE0=0 (buffer de transmisión ocupado)
	UDR0 = c;						// Cuando se desocupa, UDR0 puede recibir el nuevo dato c a trasmitir
	return 0;
}

int mi_getc0(FILE *stream)
{
	while (!(UCSR0A & (1<<RXC0)) );// Espera mientras el bit RXC0=0 (recepción incompleta)
	return UDR0;					// Cuando se completa, se lee UDR0
}
