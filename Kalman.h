/*
 * Kalman.h
 *
 * Created: 7/10/2017 12:36:44 p. m.
 *  Author: Tincho
 */ 
#include <avr/io.h>

#ifndef KALMAN_H_
#define KALMAN_H_
	float Q_angle  =  0.0004; //0.001    //0.005
	float Q_gyro   =  0.0002;  //0.003  //0.0003
	float R_angle  =  0.002;  //0.03     //0.008
	float x_bias = 0;
	float x_angle;
	float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
	float  x, S;
	float K_0, K_1;

	float W_angle  =  0.0004; //0.001    //0.005
	float W_gyro   =  0.0002;  //0.003  //0.0003
	float T_angle  =  0.002;  //0.03     //0.008
	float Y_bias = 0;
	float Y_angle;
	float P_22 = 0, P_23 = 0, P_32 = 0, P_33 = 0;
	float  y, D;
	float K_2, K_3;
	
	float kalmanCalculateX(float newAngle, float newRate, int16_t looptime)
	{
		float dt = (float)(looptime)/1000;
		x_angle += dt * (newRate - x_bias);

		//!    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
		P_00 += dt * (dt*P_11 - P_01 - P_10 + Q_angle);
		//!    P_01 +=  - dt * P_11;
		P_01 -= dt * P_11;
		//!    P_10 +=  - dt * P_11;
		P_10 -= dt * P_11;
		//!    P_11 +=  + Q_gyro * dt;
		P_11 += Q_gyro * dt;

		S = P_00 + R_angle;
		
		K_0 = P_00 / S;
		K_1 = P_10 / S;
		
		x = newAngle - x_angle;
		
		x_angle +=  K_0 * x;
		x_bias  +=  K_1 * x;

		P_00 -= K_0 * P_00;
		P_01 -= K_0 * P_01;
		P_10 -= K_1 * P_00;
		P_11 -= K_1 * P_01;
		
		return x_angle;
	}
	
	float kalmanCalculateY(float newAngle, float newRate, int16_t looptime)
	{
		float dt = (float)(looptime)/1000;
		Y_angle += dt * (newRate - Y_bias);

		//!    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
		P_22 += dt * (dt*P_33 - P_23 - P_32 + W_angle);
		//!    P_01 +=  - dt * P_11;
		P_23 -= dt * P_33;
		//!    P_10 +=  - dt * P_11;
		P_32 -= dt * P_33;
		//!    P_11 +=  + Q_gyro * dt;
		P_33 += W_gyro * dt;

		D = P_22 + T_angle;
		
		K_2 = P_22 / D;
		K_3 = P_32 /D;
		
		y = newAngle - Y_angle;
		
		Y_angle +=  K_2 * y;
		Y_bias  +=  K_3 * y;

		P_22 -= K_2 * P_22;
		P_23 -= K_2 * P_23;
		P_32 -= K_3 * P_22;
		P_33 -= K_3 * P_23;
		
		return Y_angle;
	}
#endif /* KALMAN_H_ */