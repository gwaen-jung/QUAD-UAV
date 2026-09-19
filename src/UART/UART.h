#pragma once

#include "config.h"

void UART_Init();
void UART_Update();
void UART_SendRaw(const uint8_t *buffer, uint16_t length);
