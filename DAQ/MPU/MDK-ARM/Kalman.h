#ifndef  KALMAN_H
#define KALMAN_H

typedef struct 
{
	float Q_angle  ;
	float Q_bias   ;
	float R_measure;
	float angle    ;
	float bias     ;
	float rate     ;
	float P[2][2] 	;
	
}Kalman_Angle;

void Kalamn_InitAngle(Kalman_Angle* Angle);
float Kalman_getAngle(Kalman_Angle* Angle, float newAngle, float newRate, float dt);
void Kalman_setAngle(Kalman_Angle* Angle , float NAngle);

#endif /*KALMAN_H*/