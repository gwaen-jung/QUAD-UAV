#pragma once

#include "../config.h"

void Web_Init();
void Web_Update();
void Web_SetBNOStatus(bool connected);
void Web_BroadcastLog(const char *txt);
void Web_SetPIDHandler(void (*handler)(const PID_Config &pid));
void Web_SetCommandHandler(void (*handler)(const char *command));
void Web_SetRCHandler(void (*handler)(const RC_Command &rc));