#ifndef ENCODER_H
#define ENCODER_H


#define ENCODER_CHANNEL_1 	TIM_CHANNEL_1
#define ENCODER_CHANNEL_2 	TIM_CHANNEL_2

void Encoder_Setupt(TIM_HandleTypeDef *htim);
void Encoder_Start(void);
void Encoder_MainFuction(void);
void Encoder_GetSteeringAngle(int16_t* Steering_Angle);

#endif /*ENCODER_H*/