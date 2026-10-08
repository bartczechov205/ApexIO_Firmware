#pragma once

#include <Arduino.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "ekfNavINS.h"
#include "AppResources.h"
#include "SystemDataUpdater.h"

class IMUManager
{
public:
    BNO08x myIMU;

    void processImuData(AppResources &resources,ekfNavINS &ekfNavIns,SystemDataUpdater &systemDataUpdater);
    bool begin(AppResources &resources);
    void setReports(AppResources &resources);
    void update(AppResources &resources);

private:
    imuData imuDataBox{};
    magData magDataBox{};

    bool accEnabled = false;
    bool gyroEnabled = false;
    bool magEnabled = false;

    bool AccReady = false;
    bool MagReady = false;
    bool GyroReady = false;

    uint64_t timeNow = 0;
    uint8_t eventID = 0;
};