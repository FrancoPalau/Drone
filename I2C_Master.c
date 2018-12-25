/*
 * I2C_Master.c
 *
 * Created: 3/10/2017 1:21:50 a. m.
 *  Author: Tincho
 */ 

// Rutinas basicas de i2c en modo maestro
// Originales en https://github.com/g4lvanix/I2C-master-lib
// Modificadas MyEP 2017

#ifndef  F_CPU
	#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/twi.h>

#include "i2c_master.h"

#define Prescaler 1

void i2c_init(uint32_t F_SCL)
{
	uint32_t TWBR_val;
	TWBR_val = ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2);
	TWBR = (uint8_t)TWBR_val;
}

uint8_t i2c_start(uint8_t address) // incuye start y escritura de address
{
	// reset TWI control register
	TWCR = 0;
	// transmit START condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// check if the start condition was successfully transmitted
	if((TWSR & 0xF8) != TW_START){ return 1; }
	// load slave address into data register
	TWDR = address;
	// start transmission of address
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// check if the device has acknowledged the READ / WRITE mode
	uint8_t twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
	return 0; // exito
}

uint8_t i2c_write(uint8_t data)					// escribe 1 byte al bus
{
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return 1; }
	return 0;
}

uint8_t i2c_read(uint8_t ack)					// lee 1 byte del bus
{
	// start TWI module and acknowledge data after reception
	TWCR = (1<<TWINT) | (1<<TWEN) | (ack<<TWEA);
	// wait for end of transmission
	while( !(TWCR & (1<<TWINT)) );
	// return received data from TWDR
	return TWDR;
}

void i2c_stop(void)
{
	// transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
}