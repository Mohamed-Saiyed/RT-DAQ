#ifndef	QUATERNION_H
#define QUATERNION_H

typedef enum 
{
    NONE,
    MADGWICK,
    MAHONY
}QuatFilterSel;


void QuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void mahony_updateIMU(float ax, float ay, float az, float gx, float gy, float gz, float* q);
void Mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q);
void select_filter(QuatFilterSel sel);

#endif /*QUATERNION_H*/

