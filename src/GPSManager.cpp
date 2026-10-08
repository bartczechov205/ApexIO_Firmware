#include "GPSManager.h"
#include "Macro.h"
#include "MathUtils.h"
#include <SparkFun_u-blox_GNSS_v3.h>
#include <TelemetryData.h>
#include <AppResources.h>

void GPSManager::processGpsData(AppResources &resources,ekfNavINS &ekfNavIns)
{
    update(resources);

    if (xSemaphoreTake(resources.systemStateMutex, portMAX_DELAY) == pdTRUE)
    {
        resources.validFix = getFix();

        if (xQueueReceive(resources.gpsPositionQueue, &resources.gpsPosReceived, 0) == pdTRUE)
        {
            if (resources.validFix)
            {
                ekfNavIns.gpsPosDataUpdateEKF(resources.gpsPosReceived);
            }
        }

        if (xQueueReceive(resources.gpsVelocityQueue, &resources.gpsVelReceived, 0) == pdTRUE)
        {
            if (resources.validFix)
            {
                ekfNavIns.gpsVelDataUpdateEKF(resources.gpsVelReceived);
            }
        }

        if (xQueueReceive(resources.gpsExtraQueue, &resources.gpsExtraReceived, 0) == pdTRUE)
        {
            if (resources.validFix)
            {
                resources.systemData.rawGpsExtraData = resources.gpsExtraReceived;
            }
        }

        xSemaphoreGive(resources.systemStateMutex);
    }
}

bool GPSManager::begin()
{
    GPS_SERIAL.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    while (!myGPS.begin(GPS_SERIAL))
    {
        Serial.println("Attempting to connect to GPS...");
        delay(500);
    }

    myGPS.setUART1Output(COM_TYPE_UBX);
    myGPS.setNavigationFrequency(GPS_FREQUENCY);

    return true;
}

bool GPSManager::getFix()
{
    if (myGPS.getGnssFixOk())
    {
        return true;
    }
    else
    {
        return false;
    }
}

void GPSManager::update(AppResources &resources)
{

    if (!myGPS.getPVT())
    {
        return;
    }

    bool validFix = myGPS.getGnssFixOk();

    if (xSemaphoreTake(resources.systemStateMutex, portMAX_DELAY) == pdTRUE)
    {
        resources.validFix = validFix;
        xSemaphoreGive(resources.systemStateMutex);
    }

    uint64_t timeNow = micros();
    const int32_t altitudeMm = myGPS.getAltitude();

    if (validFix)
    {
        gpsPosDataBox.pos_time = timeNow;
        gpsPosDataBox.lat = mathUtils.convertGeoToRadians(myGPS.getLatitude());
        gpsPosDataBox.lon = mathUtils.convertGeoToRadians(myGPS.getLongitude());
        gpsPosDataBox.alt = mathUtils.MiliMeterstoMeters(altitudeMm);

        gpsVelDataBox.vel_time = timeNow;
        gpsVelDataBox.vN = mathUtils.convertToMPerS(myGPS.getNedNorthVel());
        gpsVelDataBox.vE = mathUtils.convertToMPerS(myGPS.getNedEastVel());
        gpsVelDataBox.vD = mathUtils.convertToMPerS(myGPS.getNedDownVel());

        xQueueSend(resources.gpsPositionQueue, &gpsPosDataBox, 0);
        xQueueSend(resources.gpsVelocityQueue, &gpsVelDataBox, 0);
    }

    myGpsExtraData.siv = myGPS.getSIV();
    myGpsExtraData.hr = myGPS.getHour();
    myGpsExtraData.min = myGPS.getMinute();
    myGpsExtraData.sec = myGPS.getSecond();
    myGpsExtraData.ms = myGPS.getMillisecond();
    myGpsExtraData.ns = myGPS.getNanosecond();
    myGpsExtraData.alt_mm = altitudeMm;
    myGpsExtraData.hdg = myGPS.getHeading();

    xQueueSend(resources.gpsExtraQueue, &myGpsExtraData, 0);
}

