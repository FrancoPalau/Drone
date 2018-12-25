/*
 * I2C_Dev.c
 *
 * Created: 3/10/2017 1:24:45 a. m.
 *  Author: Tincho
 */ 

#include <stdint.h>
#include "i2c_master.h"
#include "i2c_DEV.h"

uint8_t DEV_rdy(uint8_t ad0)
{
      uint8_t ack;         
      ack = i2c_start((DEV_ADDR|(ad0&7))<<1);
      return !ack;  // si ack=0 exito, vuelve 1
}

/*uint8_t DEV_rdy_P(uint8_t ad0)
{
	uint8_t ack;
	ack = i2c_start((DEV_ADDR1|(ad0&7))<<1);
	return !ack;  // si ack=0 exito, vuelve 1
}*/

uint8_t DEV_write(uint8_t ad0, uint8_t address, uint8_t dato)
{
      uint8_t ack;         
      while(!DEV_rdy(ad0))
      {
       // espera hasta que el sensor 0 esté listo
      }      
      ack = i2c_start((DEV_ADDR|(ad0&7))<<1);                  // condición de START
	  i2c_write(address);           // escribe byte de dirección de registro a acceder dentro del sensor
      i2c_write(dato);              // escribe el dato en la dirección escrita 
      i2c_stop();                   // condición de STOP      
	  return ack;
}

/*uint8_t DEV_write_P(uint8_t ad0, uint8_t address, uint8_t dato)
{
	uint8_t ack;
	while(!DEV_rdy_P(ad0))
	{
		// espera hasta que el sensor 0 esté listo
	}
	ack = i2c_start((DEV_ADDR1|(ad0&7))<<1);                  // condición de START
	i2c_write(address);           // escribe byte de dirección de registro a acceder dentro del sensor
	i2c_write(dato);              // escribe el dato en la dirección escrita
	i2c_stop();                   // condición de STOP
	return ack;
}*/

uint8_t DEV_read(uint8_t ad0, uint8_t address)
{
	  uint8_t dato;
      while(!DEV_rdy(ad0))
      {
            // espera hasta que el sensor 0 esté listo
      }      
      i2c_start((DEV_ADDR|(ad0&7))<<1); //dir es 0 o 1 en MPU6050, pero reservamos 3 bits
      i2c_write(address);                     // escribe byte de dirección a acceder dentro del sensor
      i2c_start((DEV_ADDR|(ad0&7))<<1|1); //lectura
      dato  = i2c_read(0);           // lee dato de i2c
      i2c_stop();                   // condición de STOP
      return(dato);                 // devuelve dato leído      
}

/*uint8_t DEV_read_P(uint8_t ad0, uint8_t address)
{
	uint8_t dato;
	while(!DEV_rdy_P(ad0))
	{
		// espera hasta que el sensor 0 esté listo
	}
	i2c_start((DEV_ADDR1|(ad0&7))<<1); //dir es 0 o 1 en MPU6050, pero reservamos 3 bits
	i2c_write(address);                     // escribe byte de dirección a acceder dentro del sensor
	i2c_start((DEV_ADDR1|(ad0&7))<<1|1); //lectura
	dato  = i2c_read(0);           // lee dato de i2c
	i2c_stop();                   // condición de STOP
	return(dato);                 // devuelve dato leído
}*/


uint16_t DEV_read16(uint8_t ad0, uint8_t address) //lee entero 16 bits
{
	uint16_t dato;
	while(!DEV_rdy(ad0))
	{
		// espera hasta que el sensor 0 esté listo
	}
	i2c_start((DEV_ADDR|(ad0&7))<<1); //dir es 0 o 1 en MPU6050, pero reservamos 3 bits
	i2c_write(address);                     // escribe byte de dirección a acceder dentro del sensor
	i2c_start((DEV_ADDR|(ad0&7))<<1|1); //lectura
	dato = i2c_read(1);           // lee dato de i2c
	dato<<=8;
	dato|= i2c_read(0);
	i2c_stop();                   // condición de STOP
	return(dato);                 // devuelve dato leído
}

/*uint16_t DEV_read16_P(uint8_t ad0, uint8_t address) //lee entero 16 bits
{
	uint16_t dato;
	while(!DEV_rdy_P(ad0))
	{
		// espera hasta que el sensor 0 esté listo
	}
	i2c_start((DEV_ADDR1|(ad0&7))<<1); //dir es 0 o 1 en MPU6050, pero reservamos 3 bits
	i2c_write(address);                     // escribe byte de dirección a acceder dentro del sensor
	i2c_start((DEV_ADDR1|(ad0&7))<<1|1); //lectura
	dato = i2c_read(1);           // lee dato de i2c
	dato<<=8;
	dato|= i2c_read(0);
	i2c_stop();                   // condición de STOP
	return(dato);                 // devuelve dato leído
}*/

/*uint32_t  read24(uint8_t ad0, uint8_t address)
{
	uint32_t dato;
	while(!DEV_rdy_P(ad0))
	{
		// espera hasta que el sensor 0 esté listo
	}
	i2c_start((DEV_ADDR1|(ad0&7))<<1); //dir es 0 o 1 en MPU6050, pero reservamos 3 bits
	i2c_write(address);                     // escribe byte de dirección a acceder dentro del sensor
	i2c_start((DEV_ADDR1|(ad0&7))<<1|1); //lectura
	dato = i2c_read(1);           // lee dato de i2c
	dato<<=8;
	dato|= i2c_read(1);
	dato<<=8;
	dato|= i2c_read(0);
	i2c_stop();                   // condición de STOP
	return(dato);                 // devuelve dato leído
}*/

/*uint16_t read16_LE(uint8_t reg, uint8_t address) 
{
	uint16_t temp = DEV_read16_P(reg,address);
	return (int16_t)(temp >> 8) | (temp << 8);

}*/

/*int16_t readS16_LE(uint8_t reg, uint8_t address)
{
	return (int16_t)DEV_read16_P(reg,address);

}*/

/*            Devuelve el byte leído de dirección ACTUAL.
              Direcciona sensor m según pin AD0.  
              No usada en este programa.                                     */

uint8_t DEV_read_fast(uint8_t ad0) // Lee desde la posición actual.
{
	  uint8_t dato;
      while(!DEV_rdy(ad0))
      {
            // espera hasta que el sensor m esté listo
      }
      dato=i2c_read(0);             // lee dato de i2c
      i2c_stop();                   // condición de STOP     
      return(dato);      
}