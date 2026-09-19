#if defined(ARDUINO_ARCH_STM32) || defined(STM32F1)
// STM32 main - FreeRTOS tasks for UART, PID (Flight), Motor send and RF receiver

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "UART/UART.h"
#include "Packet/Packet.h"
#include "Flight/Flight.h"
#include "RF_RX/RF_RX.h"
#include "Shared/SharedData.h"

void rcFeedWatchdog();

// BluePill USART2: constructor order is RX, TX.
HardwareSerial DebugSerial(PA3, PA2);
// USART1 packet link: constructor order is RX, TX.
// NOTE: Buffer size 512 broke sync on STM32 UART parser, reverted to default
HardwareSerial FlightSerial(PA10, PA9);

// Task: UART task - read USART1 (PA10) and feed SharedData via UART_Update()
static void UART_task(void *pvParameters) {
    (void) pvParameters;
    UART_Init();
    for (;;) {
        UART_Update();
        vTaskDelay(pdMS_TO_TICKS(2)); // ~500Hz poll
    }
}

// Task: PID task - run Flight_Update() at a high rate; Flight_Update will
// internally gate on new IMU samples for derivative correctness.
static void PID_task(void *pvParameters) {
    (void) pvParameters;
    Flight_Init();
    for (;;) {
        Flight_Update();
        vTaskDelay(pdMS_TO_TICKS(2)); // aim ~500Hz
    }
}

// Task: Motor send - when Motor_Output updates, forward to ESP32 via Packet
static void MotorSend_task(void *pvParameters) {
    (void) pvParameters;
    Packet_Init();
    uint32_t lastSendMs = 0;
    for (;;) {
        Motor_Output out = GetMotorSnapshot();
        const uint32_t now = millis();
        if (out.timestamp != 0 && now - lastSendMs >= 20) {
            Packet_BuildAndSend(out);
            static uint32_t lastMotorLogMs = 0;
            if (now - lastMotorLogMs >= 500) {
                lastMotorLogMs = now;
                DEBUG_SERIAL.printf("[MOTOR->UART] m1=%u m2=%u m3=%u m4=%u\n",
                                    out.m1, out.m2, out.m3, out.m4);
            }
            lastSendMs = now;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// Task tieu thu du lieu tu NRF24 va dua cung RC command vao SharedData
static void RFProcess_task(void *pvParameters) {
    (void) pvParameters;
    for (;;) {
        if (xQueueRF == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        RFData rxData{};
        if (xQueueReceive(xQueueRF, &rxData, pdMS_TO_TICKS(100)) == pdPASS) {
            RCCommand rfCommand{};
            memcpy(&rfCommand, rxData.data, sizeof(rfCommand));

            RC_Command rc{};
            rc.throttle = rfCommand.throttle;
            rc.roll_sp = static_cast<int16_t>(map(rfCommand.roll, -1000, 1000, -350, 350));
            rc.pitch_sp = static_cast<int16_t>(map(rfCommand.pitch, -1000, 1000, -350, 350));
            rc.yaw_rate_sp = static_cast<int16_t>(map(rfCommand.yaw, -1000, 1000, -350, 350));
            rc.armed = (rfCommand.aux & RF_AUX_ARMED) ? 1 : 0;
            rc.aux = rfCommand.aux;
            rc.timestamp = millis();
            UpdateRC(rc);
            rcFeedWatchdog();
        }
    }
}

// Cac hook nay chi co tac dung khi configCHECK_FOR_STACK_OVERFLOW=2 va
// configUSE_MALLOC_FAILED_HOOK=1 (xem src/STM32FreeRTOSConfig.h). Mac dinh
// cua lib STM32duino FreeRTOS la TAT ca 2 cai nay, nen truoc gio khi 1 task
// tran stack he thong se treo im lang, khong in loi gi ca.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void) xTask;
    DEBUG_SERIAL.print(F("[FATAL] STACK OVERFLOW task: "));
    DEBUG_SERIAL.println(pcTaskName);
    DEBUG_SERIAL.flush();
    while (1) { delay(200); }
}

extern "C" void vApplicationMallocFailedHook(void) {
    DEBUG_SERIAL.println(F("[FATAL] FreeRTOS heap malloc failed (het RAM)"));
    DEBUG_SERIAL.flush();
    while (1) { delay(200); }
}

void setup() {
    // USART2: PA2=TX, PA3=RX, reserved for readable debug logs.
    DEBUG_SERIAL.begin(115200);
    delay(500);
    SharedData_Init();
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("=== STM32 FreeRTOS - Flight tasks ==="));

    // Create UART parser task
    xTaskCreate(
        UART_task,
        "UARTTask",
        384, // give slightly more stack for parsing
        NULL,
        3,
        NULL
    );

    // PID task (Flight core) - must have high priority to meet timing
    // Stack tang tu 384 -> 768 word: Flight_Update() cap phat tren stack
    // nhieu struct cuc bo (RC_Command/IMU_Data/GPS_Data co double) cong
    // voi DEBUG_SERIAL.printf() nhieu tham so (>64 byte output -> malloc
    // buffer phu), 384 word (1536 byte) la khong du va gay tran stack.
    xTaskCreate(
        PID_task,
        "PIDTask",
        768,
        NULL,
        4,
        NULL
    );

    // Motor send task - medium priority
    xTaskCreate(
        MotorSend_task,
        "MotorSend",
        256,
        NULL,
        2,
        NULL
    );

    xTaskCreate(
        RF_task,
        "RFTask",
        384,
        NULL,
        3,
        NULL
    );

    xTaskCreate(
        RFProcess_task,
        "RFProcess",
        384,
        NULL,
        3,
        NULL
    );

    DEBUG_SERIAL.println(F("Da tao xong task, khoi dong scheduler..."));
    vTaskStartScheduler();

    DEBUG_SERIAL.println(F("LOI NGHIEM TRONG: vTaskStartScheduler() thoat ra - het RAM?"));
    while (1) { delay(1000); }
}

void loop() {
    // Intentionally empty - FreeRTOS runs tasks
}

#endif