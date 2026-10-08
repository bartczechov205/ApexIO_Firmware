#include "IMUManager.h"
#include "Macro.h"
#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

void IMUManager::processImuData(AppResources &resources, ekfNavINS &ekfNavIns, SystemDataUpdater &systemDataUpdater)
{
        update(resources);

        if (xSemaphoreTake(resources.systemStateMutex, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(resources.magQueue, &resources.magReceived, 0) == pdTRUE)
            {
                if (resources.validFix)
                {
                    ekfNavIns.magDataUpdateEKF(resources.magReceived);
                }
            }

            if (xQueueReceive(resources.imuQueue, &resources.imuReceived, 0) == pdTRUE)
            {
                systemDataUpdater.updateFromImu(resources.systemData, resources.imuReceived);

                if (resources.validFix && ekfNavIns.imuDataUpdateEKF(resources.imuReceived))
                {
                    systemDataUpdater.updateFormEkf(resources.systemData, ekfNavIns);
                }
            }

            xSemaphoreGive(resources.systemStateMutex);
        }
}

void IMUManager::setReports(AppResources &resources)
{
    for (int i = 0; i < 3; i++)
    {
        if (myIMU.enableAccelerometer(10) == true)
        {
            accEnabled = true;
            break;
        }

        delay(50);
    }

    if (!accEnabled)
    {
        if (xSemaphoreTake(resources.serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("Could not enable Accelerometer after retries");
            xSemaphoreGive(resources.serialMonitorMutex);
        }
    }

    delay(50);

    for (int i = 0; i < 3; i++)
    {
        if (myIMU.enableGyro(10) == true)
        {
            gyroEnabled = true;
            break;
        }

        delay(50);
    }

    if (!gyroEnabled)
    {
        if (xSemaphoreTake(resources.serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("Could not enable Gyro after retries");
            xSemaphoreGive(resources.serialMonitorMutex);
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (myIMU.enableMagnetometer(10) == true)
        {
            magEnabled = true;
            break;
        }

        delay(50);
    }

    if (!magEnabled)
    {
        if (xSemaphoreTake(resources.serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("Could not enable Magnetometer after retries");
            xSemaphoreGive(resources.serialMonitorMutex);
        }
    }

    delay(100);
}

bool IMUManager::begin(AppResources &resources)
{
    SPI.begin(IMU_SCK, IMU_MISO, IMU_MOSI, IMU_CS);
    delay(250);

    if (myIMU.beginSPI(IMU_CS, IMU_INT, IMU_RST) == false)
    {
        if (xSemaphoreTake(resources.serialMonitorMutex, portMAX_DELAY) == pdTRUE)
        {
            Serial.println("BNO08x not detected. Check your jumpers and the hookup guide. Freezing...");
            xSemaphoreGive(resources.serialMonitorMutex);
        }
        return false;
    }

    delay(100);

    setReports(resources);

    delay(150);

    if (accEnabled && gyroEnabled && magEnabled)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void IMUManager::update(AppResources &resources)
{
    if (myIMU.wasReset())
    {
        setReports(resources);
    }

    if (!myIMU.getSensorEvent())
    {
        return;
    }

    eventID = myIMU.getSensorEventID();

    if (eventID == SENSOR_REPORTID_ACCELEROMETER)
    {
        imuDataBox.acclX = myIMU.getAccelZ();
        imuDataBox.acclY = -myIMU.getAccelY();
        imuDataBox.acclZ = myIMU.getAccelX();
        AccReady = true;
    }

    if (eventID == SENSOR_REPORTID_MAGNETIC_FIELD)
    {
        magDataBox.hX = myIMU.getMagZ();
        magDataBox.hY = myIMU.getMagY();
        magDataBox.hZ = myIMU.getMagX();
        MagReady = true;
    }

    if (eventID == SENSOR_REPORTID_GYROSCOPE_CALIBRATED)
    {
        imuDataBox.gyroX = -myIMU.getGyroZ();
        imuDataBox.gyroY = -myIMU.getGyroY();
        imuDataBox.gyroZ = myIMU.getGyroX();
        GyroReady = true;
    }

    if (MagReady && AccReady && GyroReady)
    {
        timeNow = micros();

        imuDataBox.imu_time = timeNow;
        magDataBox.mag_time = timeNow;

        xQueueSend(resources.imuQueue, &imuDataBox, 0);
        xQueueSend(resources.magQueue, &magDataBox, 0);

        AccReady = false;
        MagReady = false;
        GyroReady = false;
    }
}