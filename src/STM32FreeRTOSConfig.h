/*
 * Override cho STM32duino FreeRTOS.
 * Lib mac dinh (FreeRTOSConfig_Default.h) de configCHECK_FOR_STACK_OVERFLOW = 0
 * va khong co malloc-failed hook -> khi 1 task tran stack hoac cap phat heap
 * that bai, RAM bi ghi de am tham, scheduler crash MA KHONG BAO LOI GI CA
 * (dung hien tuong: in 1 dong debug roi treo may vinh vien).
 *
 * File nay duoc STM32FreeRTOSConfig.h (trong lib) tu dong include qua
 * __has_include(), nen chi can dat trong src/ la co hieu luc.
 */
#pragma once

#include "FreeRTOSConfig_Default.h"

// FreeRTOSConfig_Default.h define 2 macro nay khong co #ifndef guard
// nen phai undef truoc khi override.
#undef configCHECK_FOR_STACK_OVERFLOW
#undef configUSE_MALLOC_FAILED_HOOK

// Bat check tran stack (method 2: kiem tra ca pattern lap stack lan dinh cao nhat)
#define configCHECK_FOR_STACK_OVERFLOW    2

// Bat hook khi pvPortMalloc() that bai (heap FreeRTOS het cho)
#define configUSE_MALLOC_FAILED_HOOK      1