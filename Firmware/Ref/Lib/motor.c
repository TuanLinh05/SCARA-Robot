#include "motor.h"

void Stepper_Motor_Init(Step_HandleTypedef *p_Motor, TIM_HandleTypeDef *p_TIM, uint32_t Chanel,
						GPIO_TypeDef *p_GPIO, uint16_t Pin){
	p_Motor->p_Motor_TIM = p_TIM;
	p_Motor->Motor_Chanel = Chanel;
	p_Motor->p_Motor_GPIOx = p_GPIO;
	p_Motor->Motor_Dir_Pin = Pin;
}
