/*

 * Drone.c
 *
 * Created: 9/9/2017 6:09:20 p. m.
 * Author : Tincho y Franco
 */

#define F_CPU 16000000

// Calcular el valor necesario para
// el valor de coincidencia de CTC en OCR1A.
#define CTC_MATCH_OVERFLOW ((F_CPU / 1000) / 8) //2000

//Ratios de conversion
#define A_R 16384.0 // para pasar a unidades "g"
#define G_R 131.0

//Conversion de radianes a grados 180/PI
#define RAD_TO_DEG 57.295779

#define DEV_ADDR 0x68			// Direcci�n est�ndar de MPU

#define MAX_OCR 249 //Maximo de los OCR que es 2ms
#define MIN_OCR 124 //Minimo de los OCR que es 1ms

#define MINIMO_OCR 135 //Esto es la minima potencia que mueve el motor
#define MAXIMO_OCR 249

#define SENAL_REFERENCIA 180

#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <stdio.h>
#include "i2c_master.h"
#include "mpu_6050.h"
#include "i2c_DEV.h"
#include "UART_ATmega0.h"
#include <util/delay.h>
#include "Kalman.h"
#include <math.h>
#include <util/atomic.h>
#include <stdlib.h>

FILE uart_io = FDEV_SETUP_STREAM(mi_putc0, mi_getc0, _FDEV_SETUP_RW); // Declara un tipo stream de E/S

// ********** Variables del interprete ***************
uint8_t indcom;
char comando[30];
// ********** Variables del temporizador ****************
volatile unsigned long timer1_millis=0;
// ********** Variables del calculo del angulo*************
unsigned long tiempo_prevX=0, tiempo_prevY=0, tiempo_actual_X=0, tiempo_actual_Y=0;
double giro_angulo_X=0, giro_angulo_Y=0;
float error_prevX=0, error_prevY=0;
int retraso=0, variable_bool=0;
// ********** Variables del PID ***************
float PID_X=0, PID_Y=0;
float PID_P[2]={0,0};
float PID_D[2]={0,0};
float PID_I[2]={0,0};
float kp=0.3;//variar depende del drone
float kd=0.01;//variar depende del drone
float ki=0.3;//variar depende del drone
int ALTURA_INICIAL=140;
// *********** Variable de apagado/encendido ***********
int ENCENDIDO = 1;

void InterpretaComando(void)
{
	float aux=0;
	switch(comando[0])
	{
		case 'V':	// cambia duty cycle de TIMER 0 o TIMER 2
			if(comando[1])
			{
				aux = atoi(&comando[1]); //Valor en Porcentaje
				aux = (aux/100*125)+124; //Calculo de Porcentaje a OCRnx valor
				ALTURA_INICIAL = aux;
				OCR0A=OCR0B=OCR2A=OCR2B=ALTURA_INICIAL;
				printf("Duty cycle 0A=%u\r\n",ALTURA_INICIAL);//Ciclo de trabajo=((2*OCR0x)/510)*100%
			}
			break;
		case 'P':
			if (comando[1])
			{
				aux = atoi(&comando[1]);
				kp=aux/10;
				printf("el valor de kp es %.2f\n",kp);
			}
			break;
		case 'D':
		if (comando[1])
		{
			aux = atoi(&comando[1]);
			kd=aux/100;
			printf("el valor de kd es %.2f\n",kd);
		}
			break;
		case 'I':
		if (comando[1])
		{
			aux = atoi(&comando[1]);
			ki=aux/10;
			printf("el valor de ki es %.2f\n",ki);
		}
			break;
		case 'C':
			//calibracion
			OCR0A=OCR0B=OCR2A=OCR2B=MIN_OCR;
			printf("ESCs calibrados\n");
			_delay_ms(2000);
			OCR0A=OCR0B=OCR2A=OCR2B=ALTURA_INICIAL;
			_delay_ms(5000);
			break;
		case 'Z':
			// Apagado de motores
			ENCENDIDO = 0;
		default:
			break;
	}
}

ISR(USART_RX_vect)
{	char dato;
	dato=getc();
	switch(dato)
	{
		case ':':
			indcom=0;
			break;
		case 13:
			comando[indcom]=0;
			InterpretaComando();
			break;
		default:
			comando[indcom++]=dato;
			break;
	}
}

void calibracion_esc()
{
	/*
	Rutina para calibracion:
		1_ Sin conectar la bateria todavia, enviar MAX PWM a los motores
		2_ Conectar la bateria
		3_ Esperar 2 segundos
		4_ enviar MIN PWM
		5_ ESCs calibrados
	*/
	OCR0A=OCR0B=OCR2A=OCR2B=MAX_OCR;
	printf("Enchufar bateria\n");
	_delay_ms(2000);
	printf("Presiona una tecla\n");
	//el codigo continua en el interprete de comandos
}

void inicia_timers(){
	
	//Para habilitar timers
	TCCR0A|=(1<<COM0A1)|(1<<COM0B1); //Pone salidas OC0A y OC0B en clear
	TCCR0B|=(1<<CS00)|(1<<CS01); //Prescaler de 64 Tpwmpc=(510*prescaler)/FCPU
	TCCR0A|=(1<<WGM00); //Modo phase correct pwm, TOP=0xFF, conmuta con OCRnx
		
	TCCR2A|=(1<<COM2A1)|(1<<COM2B1); //Pone salidas OC2A y OC2B en clear
	TCCR2B|=(1<<CS22); //Prescaler de 64 Tpwmpc=(510*prescaler)/FCPU
	TCCR2A|=(1<<WGM20); //Modo phase correct pwm, TOP=0xFF, conmuta con OCRnx
	
	//Para salidas de timers
	DDRD|=(1<<PORTD3)|(1<<PORTD5)|(1<<PORTD6);
	DDRB|=(1<<PORTB3);

	//Para funcion temporizador
	TCCR1B |= (1 << WGM12) | (1 << CS11); // CTC mode, Clock/8
	OCR1AH = (CTC_MATCH_OVERFLOW >> 8); // Carga la parte alta del byte,
	OCR1AL = CTC_MATCH_OVERFLOW; 		// luego la baja
}

ISR (TIMER1_COMPA_vect)
{
	timer1_millis++;
}

unsigned long millis ()
{
	unsigned long millis_return;
	// Asegura que no se puede interrumpir
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		millis_return = timer1_millis;
	}
	return millis_return;
}

void orientacion_filtrocomplementario(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){

	double acel_angulo_X,acel_angulo_Y;
	double kalAngleX,kalAngleY;
	double giro_X, giro_Y;

	acel_angulo_X = (atan2 (ay, az) + 3.1416) * RAD_TO_DEG;
	giro_X = (double) gx / G_R; //velocidad angular en grados/segundos
	tiempo_actual_X=millis();
	giro_angulo_X += giro_X * ( (double) (tiempo_actual_X - tiempo_prevX)/ 1000); //1000 para segundo
	kalAngleX = kalmanCalculateX(acel_angulo_X, giro_X, (tiempo_actual_X - tiempo_prevX));

	acel_angulo_Y = (atan2 (ax, az) + 3.1416) * RAD_TO_DEG;
	giro_Y = (double) gy / G_R;
	tiempo_actual_Y=millis();
	giro_angulo_Y += giro_X * ( (double) (tiempo_actual_Y - tiempo_prevY)/ 1000);
	kalAngleY = kalmanCalculateY(acel_angulo_Y, giro_Y, (tiempo_actual_Y - tiempo_prevY));

	// El siguiente codigo asegura que
	// el pid actua despues de un tiempo_prev
	// esto es solo para las pruebas!
	retraso++;
	if(retraso > 1500 && variable_bool == 0){
		variable_bool=1;
	}
	if (variable_bool != 0){
		control_PID(kalAngleX,kalAngleY);
	}

	// Actualizamos el tiempo
	tiempo_prevX=tiempo_actual_X;
	tiempo_prevY=tiempo_actual_Y;

}

void control_PID(double anguloX, double anguloY){

	PID_X=0; //Borramos calculos anteriores
	PID_Y=0;
	float errorX,errorY;
	float dtX,dtY;

	dtX = (float)(tiempo_actual_X - tiempo_prevX)/1000;
	dtY = (float)(tiempo_actual_Y - tiempo_prevY)/1000;

	errorX=anguloX-SENAL_REFERENCIA;
	errorY=anguloY-SENAL_REFERENCIA;

	// Parte proporcional
	PID_P[0]=kp*errorX;
	PID_P[1]=kp*errorY;
	// Parte derivativa
	PID_D[0]=kd*((errorX-error_prevX)/dtX);
	PID_D[1]=kd*((errorY-error_prevY)/dtY);
	// Parte integral
	PID_I[0]=PID_I[0]+(ki*errorX*dtX);
	PID_I[1]=PID_I[1]+(ki*errorY*dtY);

	
	if (-5 < errorX && errorX < 5)
	{
		PID_X=PID_P[0]+PID_D[0]+PID_I[0];
	}else{
		PID_X=PID_P[0]+PID_D[0];
	}
	if (-5 < errorY && errorY < 5)
	{
		PID_Y=PID_P[1]+PID_D[1]+PID_I[1];
	}else{
		PID_Y=PID_P[1]+PID_D[1];
	}
	
	// Saturamos los PID para que no
	// tomen valores excesivos
	if(PID_X < -50){PID_X=-50;}
	if(PID_X > 50) {PID_X=50;}
	if(PID_Y < -50){PID_Y=-50;}
	if(PID_Y > 50) {PID_Y=50;}

	int Variable_OCR0A=ALTURA_INICIAL+PID_X-PID_Y;
	int Variable_OCR2A=ALTURA_INICIAL+PID_X+PID_Y;
	int Variable_OCR0B=ALTURA_INICIAL-PID_X-PID_Y;
	int Variable_OCR2B=ALTURA_INICIAL-PID_X+PID_Y;

	// Mantenemos a los OCRnx dentro de sus rangos de operacion
	if(Variable_OCR0A < MINIMO_OCR){Variable_OCR0A = MINIMO_OCR;}
	if(Variable_OCR0A > MAXIMO_OCR){Variable_OCR0A = MAXIMO_OCR;}
	if(Variable_OCR0B < MINIMO_OCR){Variable_OCR0B = MINIMO_OCR;}
	if(Variable_OCR0B > MAXIMO_OCR){Variable_OCR0B = MAXIMO_OCR;}
	if(Variable_OCR2A < MINIMO_OCR){Variable_OCR2A = MINIMO_OCR;}
	if(Variable_OCR2A > MAXIMO_OCR){Variable_OCR2A = MAXIMO_OCR;}
	if(Variable_OCR2B < MINIMO_OCR){Variable_OCR2B = MINIMO_OCR;}
	if(Variable_OCR2B > MAXIMO_OCR){Variable_OCR2B = MAXIMO_OCR;}

	// Actualizamos los OCRnx
	OCR0A=Variable_OCR0A;
	OCR0B=Variable_OCR0B;
	OCR2A=Variable_OCR2A;
	OCR2B=Variable_OCR2B;
	
	printf("%.2f, %.2f, %.2f, %.2f\n",anguloX,anguloY,PID_X,PID_Y);

	// Actualizamos el error
	error_prevX=errorX;
	error_prevY=errorY;
}

void apagar_motores(void){
	OCR0A=OCR0B=OCR2A=OCR2B=0;
}

void iniciar_motores(void){
	_delay_ms(2000); //Espera de seguridad por la calibracion
	OCR0A=OCR0B=OCR2A=OCR2B=ALTURA_INICIAL; //Inicializamos los motores con una potencia minima
	_delay_ms(5000);
}

int main(void)
{
	int16_t ax, ay, az, gx, gy, gz;

	i2c_init(100000UL);

	_delay_ms(10);

	DEV_write(0,MPU6050_RA_PWR_MGMT_1, 0x0B);		// Activa MPU
	DEV_write(0,MPU6050_RA_CONFIG, 0x05);			// filtro LP 10hz

	mi_UART_Init0(57600,0,0);
	stdout = stdin = &uart_io;  // El stream (FILE) uart_io es la E/S est�ndar, es decir para putc y getc
	_delay_ms(1000);

	inicia_timers();
	calibracion_esc();
	//iniciar_motores();

	//Habilitar interrupcion por compare match
	TIMSK1 |= (1 << OCIE1A);

	sei();
	UCSR0B|= (1<<RXCIE0); // Habilita interrupciones de UART

	_delay_ms(500);

	tiempo_prevX=millis();
	tiempo_prevY=millis();

	while (ENCENDIDO)
	{
		ax = DEV_read16(0,MPU6050_RA_ACCEL_XOUT_H);
		ay = DEV_read16(0,MPU6050_RA_ACCEL_YOUT_H);
		az = DEV_read16(0,MPU6050_RA_ACCEL_ZOUT_H);
		gx = DEV_read16(0,MPU6050_RA_GYRO_XOUT_H);
		gy = DEV_read16(0,MPU6050_RA_GYRO_YOUT_H);
		gz = DEV_read16(0,MPU6050_RA_GYRO_ZOUT_H);
		
		orientacion_filtrocomplementario(ax,ay,az,gx,gy,gz);
	}

	// Se apagan los motores
	apagar_motores();
}
