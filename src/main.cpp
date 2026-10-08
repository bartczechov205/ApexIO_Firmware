#include <Arduino.h>
#include "Macro.h"
#include "SharedBegin.h"
#include "ekfNavINS.h"
#include <TFT_eSPI.h>
#include "AppStateManager.h"
#include "TelemetryEngine.h"
#include "TFTManager.h"
#include "SystemDataUpdater.h"
#include "AppResources.h"
#include "SDManager.h"

SharedBegin sharedBegin;
IMUManager imuManager;
GPSManager gpsManager;
ekfNavINS ekfNavIns;
AppStateManager appStateManager;
TelemetryEngine telemetryEngine;
TFTManager tftManager;
SDManager sdManager;

SystemDataUpdater systemDataUpdater;
AppResources resources;

void IRAM_ATTR imuInt()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(resources.imuSemaphore, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vTaskIMU(void *pvParameters)
{
    for (;;)
    {
        imuManager.processImuData(resources,ekfNavIns,systemDataUpdater);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void vTaskGPS(void *pvParameters)
{
    for (;;)
    {
        gpsManager.processGpsData(resources, ekfNavIns);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void vTaskAppState(void *pvParametrs)
{
    bool validFix = false;
    bool canContinue = false;

    for (;;)
    {
        if (xSemaphoreTake(resources.systemStateMutex, portMAX_DELAY) == pdTRUE)
        {
            validFix = resources.validFix;

            if (resources.validFix && resources.systemReady)
            {
                canContinue = true;
            }
            else
            {
                canContinue = false;
            }
            xSemaphoreGive(resources.systemStateMutex);
        }

        appStateManager.updateState(validFix, canContinue);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void vTaskTelemetryEngine(void *pvParameters)
{
    for (;;)
    {
        if (xSemaphoreTake(resources.systemStateMutex, portMAX_DELAY) == pdTRUE)
        {
            telemetryEngine.updateTelemetryEngine(resources.systemData);
            xSemaphoreGive(resources.systemStateMutex);
        }

        telemetryEngine.processDataForState(appStateManager);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTaskTFT(void *pvParametrs)
{
    DisplayResources displayResources;

    tftManager.drawSplashScreen();

    displayResources.startTime = millis();

    for (;;)
    {
        tftManager.updateDisplay(resources,appStateManager,displayResources);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void vTaskSD(void *pvParameters)
{
    SDData sdReceived;

    bool isFileOpen = false;

    if (!sdManager.begin())
    {
        vTaskDelete(NULL);
    }

    for (;;)
    {
        if (xQueueReceive(resources.lapTimerSDQueue, &sdReceived, portMAX_DELAY) == pdTRUE)
        {
            sdManager.processSdRecord(sdReceived, isFileOpen);
        }
    }
}

void setup()
{
    digitalWrite(TFT_BL, HIGH);
    pinMode(TFT_BL, OUTPUT);

    pinMode(SD_DETECT_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(200);

    resources.imuSemaphore = xSemaphoreCreateBinary();
    resources.systemStateMutex = xSemaphoreCreateMutex();
    resources.serialMonitorMutex = xSemaphoreCreateMutex();

    resources.imuQueue = xQueueCreate(10, sizeof(imuData));
    resources.magQueue = xQueueCreate(10, sizeof(magData));
    resources.gpsPositionQueue = xQueueCreate(10, sizeof(gpsPosData));
    resources.gpsVelocityQueue = xQueueCreate(10, sizeof(gpsVelData));
    resources.gpsExtraQueue = xQueueCreate(10, sizeof(gpsExtraData));
    resources.lapTimerSDQueue = xQueueCreate(128, sizeof(SDData));

    resources.axisCalibQueue = xQueueCreate(1, sizeof(AxisCalibData));
    resources.liveTelemetryQueue = xQueueCreate(1, sizeof(LiveTeleData));
    resources.dragModeQueue = xQueueCreate(1, sizeof(DragModeData));
    resources.lapTimerQueue = xQueueCreate(1, sizeof(LapTimerData));

    if (!sharedBegin.begin(resources))
    {
        return;
    }

    attachInterrupt(digitalPinToInterrupt(IMU_INT), imuInt, FALLING);

    xTaskCreatePinnedToCore(vTaskIMU, "vTaskIMU", 16384, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(vTaskGPS, "vTaskGPS", 16384, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(vTaskAppState, "vTaskAppState", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskTFT, "vTaskTFT", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskTelemetryEngine, "vTaskTelemetryEngine", 8192, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSD, "vTaskSD", 8192, NULL, 1, NULL, 1);

    delay(500);
}

void loop() {}