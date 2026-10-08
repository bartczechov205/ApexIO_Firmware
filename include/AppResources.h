#pragma once

#include <Arduino.h>
#include "TelemetryData.h"

struct AppResources
{
    QueueHandle_t imuQueue;
    QueueHandle_t magQueue;
    QueueHandle_t gpsPositionQueue;
    QueueHandle_t gpsVelocityQueue;
    QueueHandle_t gpsExtraQueue;
    QueueHandle_t sdQueue;

    QueueHandle_t axisCalibQueue;
    QueueHandle_t liveTelemetryQueue;
    QueueHandle_t dragModeQueue;
    QueueHandle_t lapTimerQueue;
    QueueHandle_t lapTimerSDQueue;

    SemaphoreHandle_t imuSemaphore;
    SemaphoreHandle_t systemStateMutex;
    SemaphoreHandle_t serialMonitorMutex;

    SystemData systemData{};

    imuData imuReceived{};
    magData magReceived{};
    gpsPosData gpsPosReceived{};
    gpsVelData gpsVelReceived{};
    gpsExtraData gpsExtraReceived{};

    int8_t utcOffset = 0;
    int8_t startLineWidth = 10;
    int8_t gateWidth = 10;
    
    float deltaMax = 2.0f;

    bool validFix = false;
    bool systemReady = false;
};
