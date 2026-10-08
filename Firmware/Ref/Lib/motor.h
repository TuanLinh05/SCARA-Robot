#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f1xx_hal.h"

typedef enum{
	DIR_FRW=0,
	DIR_BKW,
	DIR_IDLE
}DIR_State;


typedef struct{
	TIM_HandleTypeDef *p_Motor_TIM;
	uint32_t Motor_Chanel;

	GPIO_TypeDef *p_Motor_GPIOx;
	uint16_t Motor_Dir_Pin;

}Step_HandleTypedef;

void Stepper_Motor_Init(Step_HandleTypedef *p_Motor, TIM_HandleTypeDef *p_TIM, uint32_t Chanel,
						GPIO_TypeDef *p_GPIO, uint16_t Pin);


#endif
