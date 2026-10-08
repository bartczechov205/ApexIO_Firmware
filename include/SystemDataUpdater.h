#pragma once

#include "TelemetryData.h"
#include "ekfNavINS.h"

class SystemDataUpdater
{
public:

void updateFormEkf(SystemData &data, ekfNavINS &ekf);
void updateFromImu(SystemData &data, imuData &imu);
};