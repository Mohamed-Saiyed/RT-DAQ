#include "Kalman.h"

void Kalamn_InitAngle(Kalman_Angle* Angle)
{
	Angle-> Q_angle  = 0.001f;
	Angle-> Q_bias   = 0.003f;
	Angle-> R_measure= 0.03f;
	Angle-> angle    = 0.0f;
	Angle-> bias     = 0.0f;
	Angle-> rate     = 0;
	Angle->P[0][0]	 = 0;
	Angle->P[0][1]	 = 0;
	Angle->P[1][0]	 = 0;
	Angle->P[1][1]	 = 0;
}


float Kalman_getAngle(Kalman_Angle* Angle, float newAngle, float newRate, float dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
     Angle->rate = newRate -  Angle->bias;
     Angle->angle += dt *  Angle->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    Angle->P[0][0] += dt * (dt* Angle->P[1][1] -  Angle->P[0][1] -  Angle->P[1][0] +  Angle->Q_angle);
	Angle->P[0][1] -= dt *  Angle->P[1][1];
	Angle->P[1][0] -= dt *  Angle->P[1][1];
	Angle->P[1][1] +=  Angle->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S =  Angle->P[0][0] +  Angle->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] =  Angle->P[0][0] / S;
    K[1] =  Angle->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle -  Angle->angle; // Angle difference
    /* Step 6 */
     Angle->angle += K[0] * y;
     Angle->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp =  Angle->P[0][0];
    float P01_temp =  Angle->P[0][1];

     Angle->P[0][0] -= K[0] * P00_temp;
     Angle->P[0][1] -= K[0] * P01_temp;
     Angle->P[1][0] -= K[1] * P00_temp;
     Angle->P[1][1] -= K[1] * P01_temp;

    return  Angle->angle;
}

void Kalman_setAngle(Kalman_Angle* Angle , float NAngle)
{
	Angle->angle = NAngle;
} // Used to set angle, this should be set as the starting angle
