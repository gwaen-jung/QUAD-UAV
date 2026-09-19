#pragma once

#include "config.h"

void InitSharedData();
void SharedData_Init();

IMU_Data GetIMUSnapshot();
void UpdateIMU(const IMU_Data &newValue);

RC_Command GetRCSnapshot();
void UpdateRC(const RC_Command &newValue);

Motor_Output GetMotorSnapshot();
void UpdateMotor(const Motor_Output &newValue);

GPS_Data GetGPSSnapshot();
void UpdateGPS(const GPS_Data &newValue);

System_Status GetSystemStatus();
void UpdateSystemStatus(const System_Status &newValue);
