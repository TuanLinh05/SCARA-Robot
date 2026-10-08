#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "scara.h"
#include "Ring_Buffer.h"

void COMMAND_LINE_Init();
void COMMAND_LINE_Handle();
void COMMAND_LINE_Receive(uint8_t rx_data);

void Run_theta(Scara_Theta *Scara_Theta, float despos);
void Push_cmd_to_Ring_Buffer(uint8_t *cmd, uint8_t *object, float val);
void respond_printf(const char* str,...);
#endif
