#include "SystemDataUpdater.h"

void SystemDataUpdater::updateFormEkf(SystemData &data, ekfNavINS &ekf)
{
   data.ekfOutputData.vN = ekf.getVelNorth_ms();
   data.ekfOutputData.vE = ekf.getVelEast_ms();
   data.ekfOutputData.vD = ekf.getVelDown_ms();
   data.ekfOutputData.lat_rad = ekf.getLatitude_rad();
   data.ekfOutputData.lon_rad = ekf.getLongitude_rad();
   data.ekfOutputData.alt_m = ekf.getAltitude_m(); 
   data.ekfOutputData.hdg_rad = ekf.getHeading_rad();
}

void SystemDataUpdater::updateFromImu(SystemData &data, imuData &imu)
{
    data.rawImuData.acclX = imu.acclX;
    data.rawImuData.acclY = imu.acclY;
    data.rawImuData.acclZ = imu.acclZ;
    data.rawImuData.gyroX = imu.gyroX;
    data.rawImuData.gyroY = imu.gyroY;
    data.rawImuData.gyroZ = imu.gyroZ;
}





