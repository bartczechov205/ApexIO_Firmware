#pragma once

#include <Arduino.h>
#include "Macro.h"
#include "GPSManager.h"
#include "IMUManager.h"
#include "AppStateManager.h"
#include "TFTManager.h"

extern IMUManager imuManager;
extern GPSManager gpsManager;
extern TFTManager tftManager;
extern AppStateManager appStateManager;

class SharedBegin
{
public:
    bool begin(AppResources &resources)
    {
        tftManager.begin();

        bool myGpsReady = gpsManager.begin();
        bool myImuReady = imuManager.begin(resources);

        pinMode(BTN_UP, INPUT_PULLUP);
        pinMode(BTN_DOWN, INPUT_PULLUP);
        pinMode(BTN_OK, INPUT_PULLUP);
        pinMode(BTN_BACK, INPUT_PULLUP);

        if (!Wire.begin(I2C_SDA, I2C_SCL))
        {
            Serial.println("INA219 connection failed");
        }

        if (myGpsReady && myImuReady)
        {
            if (xSemaphoreTake(resources.serialMonitorMutex, portMAX_DELAY) == pdTRUE)
            {
                Serial.println("Configuration completed successfully");
                xSemaphoreGive(resources.serialMonitorMutex);
            }

            appStateManager.begin();
            return true;
        }

        if (xSemaphoreTake(resources.serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            if (!myGpsReady)
            {
                Serial.println("GPS connection failed");
            }
            if (!myImuReady)
            {
                Serial.println("IMU connection failed");
            }
            xSemaphoreGive(resources.serialMonitorMutex);
        }

        return false;
    }
};
