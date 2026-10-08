#include <math.h>
#include <scara.h>

#define DEG_PER_PULSE	0.225f
#define DIS_PER_PULSE	0.00125f
#define PERCENT_DIS_ACC 0.3f
#define PERCENT_DIS_DE	0.3f
#define DELTA_VEL_ARR   800 // Convert value to ARR register: 1Hz->5kHz => 1000->200

#define PULSE_PER_TICK	8		//Revoltute: 8 pulse <=> 3.6 degree, update 1 time velocity
#define PULSE_PER_TICK2 400		//Prismatic: 400 pulse <=> 1mm, update 1 time velocity

void Init_Theta_Joint(Scara_Theta *Theta,
					  TIM_HandleTypeDef *p_TIM,
					  uint32_t Chanel,
					  GPIO_TypeDef *p_GPIOx,
					  uint16_t Pin,
					  JOINT_Type Type){
	Theta->Joint.p_Motor_TIM = p_TIM;
	Theta->Joint.Motor_Chanel = Chanel;
	Theta->Joint.p_Motor_GPIOx = p_GPIOx;
	Theta->Joint.Motor_Dir_Pin = Pin;
	Theta->Type = Type;
	Theta->cnt_pulse = 0;
	Theta->Vel.ARR_val=999;
	Theta->Dir = DIR_IDLE;
}

void Set_Joint_Speed(Scara_Theta *Theta, uint32_t speed){
	__HAL_TIM_SET_AUTORELOAD(Theta->Joint.p_Motor_TIM, speed);
	__HAL_TIM_SET_COMPARE(Theta->Joint.p_Motor_TIM, Theta->Joint.Motor_Chanel,(uint32_t)speed/2);
}

void Trapezoidal_Profile(float curpos, float despos, Vel_t *Vel, JOINT_Type type){

	float N = fabs(despos - curpos);
	if(N == 0.0f){
		Vel->N_step = 0;
		Vel->Acc_step_index = 0;
		Vel->De_step_index = 0;
		return;
	}

	switch (type) {
		case JOINT_REVOLUTE_3:
			Vel->N_step =(uint32_t)(N*RATIO_JOINT_3/(DEG_PER_PULSE));
			break;
		case JOINT_REVOLUTE_9:
			Vel->N_step =(uint32_t)(N*RATIO_JOINT_9/(DEG_PER_PULSE));
			break;
		case JOINT_PRISMATIC:
			Vel->N_step =(uint32_t)(N/DIS_PER_PULSE);
			break;
		default:
			break;
	}
	if(Vel->N_step*PERCENT_DIS_ACC < DELTA_VEL_ARR){
		Vel->Acc_step_index = (uint32_t) Vel->N_step*PERCENT_DIS_ACC;
	} else{
		Vel->Acc_step_index = DELTA_VEL_ARR;
	}
	Vel->De_step_index = Vel->N_step - Vel->Acc_step_index;


}

void Handle_Theta(Scara_Theta *Theta){
	if(Theta->Vel.N_step > 0){
		if(Theta->cnt_pulse < Theta->Vel.Acc_step_index){
			Set_Joint_Speed(Theta, Theta->Vel.ARR_val);
			Theta->Vel.ARR_val--;
		}else if(Theta->cnt_pulse > Theta->Vel.De_step_index){
			Set_Joint_Speed(Theta, Theta->Vel.ARR_val);
			Theta->Vel.ARR_val++;
		}
		Theta->cnt_pulse++;
		if(Theta->cnt_pulse == Theta->Vel.N_step) {
			HAL_TIM_PWM_Stop_IT(Theta->Joint.p_Motor_TIM, Theta->Joint.Motor_Chanel);
			Theta->Cur_Theta = Theta->Des_Theta;
			Theta->cnt_pulse = 0;
			Theta->Dir = DIR_IDLE;
			return;//reach target
		}
	}

}
