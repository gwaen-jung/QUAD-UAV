#ifndef GPS_H
#define GPS_H

#include "config.h"

#if defined(ARDUINO_ARCH_STM32)
#include <STM32FreeRTOS.h>
#endif

void GPS_Init();
void GPS_Update();
void GPS_StartTask();

extern TaskHandle_t GPS_handle;

#endif
