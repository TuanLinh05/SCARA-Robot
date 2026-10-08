#include "Command_Line.h"
#include "Command_Line_Type.h"

#include "Ring_Buffer.h"
#include <stdarg.h>
#include "stm32f1xx_hal.h"

uint8_t cl_buf[255];
uint8_t cl_pointer = 0;
uint8_t cl_flag = 0;
extern UART_HandleTypeDef huart2;
extern Scara_Theta Theta1;
extern Scara_Theta Theta2;
extern Scara_Theta Theta3;
extern RING_BUFFER_HandleTypeDef ring_buffer_theta1;
extern RING_BUFFER_HandleTypeDef ring_buffer_theta2;
extern RING_BUFFER_HandleTypeDef ring_buffer_theta3;
cli_command_info_t cli_command_info[]={
		{"SET", Push_cmd_to_Ring_Buffer},
		{"GET",},
		{NULL,NULL}
};
void COMMAND_LINE_Receive(uint8_t rx_data)
{
	if(rx_data != ';')
	{
		cl_buf[cl_pointer++] = rx_data;
	}
	else
	{
		cl_buf[cl_pointer] = '\0';
		cl_flag = 1;
	}
}

void COMMAND_LINE_Init()
{

}

cli_command_info_t *find_command_infor(uint8_t *cmd){
	cli_command_info_t *command=cli_command_info;
	while(command != NULL){
		if(strcmp(command->cmd_name,cmd)==0){
			return command;
		}
		command++;
	}
}
void Push_cmd_to_Ring_Buffer(uint8_t *cmd, uint8_t *object, float val){
	Packed_cmd_t Packed_cmd={cmd,object,val};
	if(strcmp(object,"X")==0) RING_BUFFER_Push(&ring_buffer_theta1, Packed_cmd);
	if(strcmp(object,"Y")==0) RING_BUFFER_Push(&ring_buffer_theta2, Packed_cmd);
	if(strcmp(object,"Z")==0) RING_BUFFER_Push(&ring_buffer_theta3, Packed_cmd);
}

void Cmd_control_handle(RING_BUFFER_HandleTypeDef *ring_buffer,Scara_Theta *Theta){
	if(RING_BUFFER_Available(ring_buffer) && Theta->Dir == DIR_IDLE ){
		Packed_cmd_t Packed_cmd;
		RING_BUFFER_Pop(ring_buffer, &Packed_cmd);
		Run_theta(Theta, Packed_cmd.value);
	}
}
void Run_theta(Scara_Theta *Scara_Theta, float despos){
	Trapezoidal_Profile(Scara_Theta->Cur_Theta,despos,&(Scara_Theta->Vel),Scara_Theta->Type);
	Scara_Theta->Des_Theta = despos;
	if(Scara_Theta->Vel.N_step > 0){						//Set direction of theta
		if(Scara_Theta->Des_Theta > Scara_Theta->Cur_Theta){
			HAL_GPIO_WritePin(Scara_Theta->Joint.p_Motor_GPIOx, Scara_Theta->Joint.Motor_Dir_Pin, GPIO_PIN_SET);
			Scara_Theta->Dir = DIR_BKW;
		}else{
			HAL_GPIO_WritePin(Scara_Theta->Joint.p_Motor_GPIOx, Scara_Theta->Joint.Motor_Dir_Pin, GPIO_PIN_RESET);
			Scara_Theta->Dir = DIR_FRW;
		}
		Scara_Theta->Joint.p_Motor_TIM->Instance->CNT =0;//reset timer
		HAL_TIM_PWM_Start_IT(Scara_Theta->Joint.p_Motor_TIM, Scara_Theta->Joint.Motor_Chanel);
	}
}
void COMMAND_LINE_Handle(){
	if(cl_flag)
	{
		char *arg_list[10];
		uint8_t arg_num = 0;

		char *temp_token = strtok((char *)cl_buf, ",");
		while(temp_token != NULL)
		{
			arg_list[arg_num++] = temp_token;
			temp_token = strtok(NULL, ",");
		}

//		if(strstr(arg_list[0], "SET") != NULL)
//		{
////			tx_frame_len = arg_num - 1;
//
//
////			if(tx_frame_data[1] == 0)
////			{
////				uint8_t temp_str0[] = {CLOSE_VALVE};
////				Push_Command_To_Ring_Buffer(tx_frame_data[0], 1, 7, 1, sizeof(temp_str0), temp_str0);
////			}
//
//		}
		cli_command_info_t *command = find_command_infor((uint8_t*)arg_list[0]);
		if(command != NULL){
			float val=atof(arg_list[2]);
			command->func(arg_list[0], arg_list[1],val);
		}
		cl_pointer = 0;
		cl_flag = 0;
	}
}

void respond_printf(const char* str,...){
	char *sArray=NULL;

	va_list args;
	va_start(args, str);
	vasprintf(&sArray, str, args);
	va_end(args);
	if(sArray != NULL){
		HAL_UART_Transmit(&huart2,(uint8_t *) sArray, strlen(sArray), 100);
		free(sArray);
	}
}
