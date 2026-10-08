#ifndef COMMAND_LINE_TYPE_H
#define COMMAND_LINE_TYPE_H

#include <stdint.h>
#include "scara.h"
//#include "Ring_Buffer.h"
typedef void(*cli_command_func_t)(uint8_t *cmd, uint8_t *object, float val);

typedef struct{
	uint8_t *cmd_name;
	cli_command_func_t func;
}cli_command_info_t;

typedef struct{
	uint8_t *cmd_name;
	uint8_t *object;
	float value;
} Packed_cmd_t;

#endif
