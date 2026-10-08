#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "ekfNavINS.h"
#include "AppResources.h"
#include "TelemetryData.h"
#include "MathUtils.h"

class GPSManager
{
public:
    void processGpsData(AppResources &resources,ekfNavINS &ekfNavIns);
    bool begin();
    bool getFix();
    void update(AppResources &resources);

private:
    SFE_UBLOX_GNSS_SERIAL myGPS;
    gpsExtraData myGpsExtraData;
    gpsPosData gpsPosDataBox;
    gpsVelData gpsVelDataBox;
    MathUtils mathUtils;
};
