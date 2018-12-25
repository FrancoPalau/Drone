/*
 * I2C_Dev.h
 *
 * Created: 3/10/2017 1:23:53 a. m.
 *  Author: Tincho
 */ 

#ifndef DEV_ADDR
	#define DEV_ADDR 0x68//Direccion para iniciar MPU2560
	//#define DEV_ADDR1 0x76//Direccion para iniciar BMP280
#endif

#include <stdint.h>
uint8_t DEV_rdy(uint8_t ad0);
//uint8_t DEV_rdy_P(uint8_t ad0);
uint8_t DEV_write(uint8_t ad0, uint8_t address, uint8_t dato);
//uint8_t DEV_write_P(uint8_t ad0, uint8_t address, uint8_t dato);
uint8_t DEV_read(uint8_t ad0, uint8_t address);
//uint8_t DEV_read_P(uint8_t ad0, uint8_t address);
uint16_t DEV_read16(uint8_t ad0, uint8_t address); //lee entero 16 bits;
//uint16_t DEV_read16_P(uint8_t ad0, uint8_t address); //lee entero 16 bits;
//uint16_t read16_LE(uint8_t reg, uint8_t address); // little endian
//int16_t readS16_LE(uint8_t reg, uint8_t address);
//uint32_t read24(uint8_t ad0, uint8_t address);
uint8_t DEV_read_fast(uint8_t ad0);