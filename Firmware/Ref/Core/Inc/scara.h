#ifndef SCARA_H
#define SCARA_H

#include "motor.h"

#define PULSE_PER_ROUND 1600
#define RATIO_JOINT_3		3
#define RATIO_JOINT_9		9
#define DIS_PER_ROUND   2

typedef enum{
	JOINT_PRISMATIC,
	JOINT_REVOLUTE_3,
	JOINT_REVOLUTE_9
}JOINT_Type;

typedef struct{
	uint32_t N_step;
	uint32_t Acc_step_index;
	uint32_t De_step_index;
	volatile uint32_t cnt_vel;
	volatile uint32_t ARR_val;
}Vel_t;

typedef struct{

	Step_HandleTypedef Joint;
	DIR_State Dir;
	float Des_Theta;
	float Cur_Theta;
	JOINT_Type Type;
	volatile uint32_t cnt_pulse;
//	uint32_t *p_ARR_vel;
	Vel_t Vel;
}Scara_Theta;

//typedef struct{
//	Step_HandleTypedef Prismatic_Joint;
//	DIR_State Dir;
//	float Des_Delta;
//	float Cur_Delta;
//	uint32_t *p_ARR_vel;
//}Scara_Delta;

void Init_Theta_Joint(Scara_Theta *Theta,
					  TIM_HandleTypeDef *p_TIM,
					  uint32_t Chanel,
					  GPIO_TypeDef *p_GPIOx,
					  uint16_t Pin,
					  JOINT_Type Type);

void Set_Joint_Speed(Scara_Theta *Theta, uint32_t speed);

void Trapezoidal_Profile(float curpos, float despos, Vel_t *Vel, JOINT_Type type);
void Handle_Theta(Scara_Theta *Theta);
#endif
