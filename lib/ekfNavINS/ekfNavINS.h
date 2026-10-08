#pragma once

#undef B0
#undef B1

#include <stdint.h>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

constexpr float TAU_A = 100.0f;
constexpr float TAU_G = 50.0f;
constexpr float P_P_INIT = 10.0f;
constexpr float P_V_INIT = 1.0f;
constexpr float P_A_INIT = 0.34906f;
constexpr float P_HDG_INIT = 3.14159f;
constexpr float P_AB_INIT = 0.9810f;
constexpr float P_GB_INIT = 0.01745f;
constexpr float G = 9.807f;
constexpr double ECC2 = 0.0066943799901;
constexpr double EARTH_RADIUS = 6378137.0;

struct gpsPosData {
    uint64_t pos_time;
    double lat;
    double lon;
    double alt;
};

struct gpsVelData {
    uint64_t vel_time;
    double vN;
    double vE;
    double vD;
};

struct imuData {
    uint64_t imu_time;
    float gyroX;
    float gyroY;
    float gyroZ;
    float acclX;
    float acclY;
    float acclZ;
};

struct magData {
    uint64_t mag_time;
    float hX;
    float hY;
    float hZ;
};

struct ekfState {
    uint64_t timestamp;
    Eigen::Vector3d lla;
    Eigen::Vector3d velNED;
    Eigen::Matrix<float,3,1> linear;
    Eigen::Matrix<float,3,1> angular;
    Eigen::Matrix<float,4,1> quat;
    Eigen::Vector3d accl_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Matrix<float, 15, 15> cov;
};

class ekfNavINS {
  public:
    void ekf_update(uint64_t time, 
                    double vn, double ve, double vd,    
                    double lat, double lon, double alt, 
                    float p, float q, float r,          
                    float ax, float ay, float az,       
                    float hx, float hy, float hz,
                    bool applyGpsCorrection = true);

    bool initialized()          { return initialized_; }
    bool gpsPositionReady()     { return is_gps_pos_initialized; }
    bool gpsVelocityReady()     { return is_gps_vel_initialized; }
    bool imuReady()             { return is_imu_initialized; }
    bool magnetometerReady()    { return is_mag_initialized; }
    
    float getPitch_rad()        { return theta; }
    float getRoll_rad()         { return phi; }
    float getHeadingConstrainAngle180_rad() { return constrainAngle180(psi); }
    float getHeading_rad()      { return psi; }
    
    double getLatitude_rad()    { return lat_ins; }
    double getLongitude_rad()   { return lon_ins; }
    double getAltitude_m()      { return alt_ins; }
    double getVelNorth_ms()     { return vn_ins; }
    double getVelEast_ms()      { return ve_ins; }
    double getVelDown_ms()      { return vd_ins; }
    float getGroundTrack_rad()  { return atan2f((float)ve_ins,(float)vn_ins); }
    
    float getGyroBiasX_rads()   { return gbx; }
    float getGyroBiasY_rads()   { return gby; }
    float getGyroBiasZ_rads()   { return gbz; }
    float getAccelBiasX_mss()   { return abx; }
    float getAccelBiasY_mss()   { return aby; }
    float getAccelBiasZ_mss()   { return abz; }

    std::tuple<float,float,float> getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz);

    bool imuDataUpdateEKF(const imuData& imu, ekfState* ekfOut = nullptr);
    void magDataUpdateEKF(const magData& mag);
    void gpsPosDataUpdateEKF(const gpsPosData& pos);
    void gpsVelDataUpdateEKF(const gpsVelData& vel);

    void setAcclNoise (float acclNoise) { SIG_W_A = acclNoise; }
    void setAcclBias  (float acclBias)  { SIG_A_D = acclBias;  }
    void setGyroNoise (float gyroNoise) { SIG_W_G = gyroNoise; }
    void setGyroBias  (float gyroBias)  { SIG_G_D = gyroBias;  }
    void setStdDevGpsPosNE(float stdDevGpsPosNE) { SIG_GPS_P_NE = stdDevGpsPosNE; }
    void setStdDevGpsPosD(float stdDevGpsPosD)   { SIG_GPS_P_D  = stdDevGpsPosD;  }
    void setStdDevGpsVelNE(float stdDevGpsVelNE) { SIG_GPS_V_NE = stdDevGpsVelNE; }
    void setStdDevGpsVelD(float stdDevGpsVelD)   { SIG_GPS_V_D  = stdDevGpsVelD;  }
    void setStdDevMag(float stdDevMag)           { SIG_MAG      = stdDevMag;      }

  private:
    float SIG_W_A = 0.30f;       
    float SIG_A_D = 0.01f;       
    float SIG_W_G = 0.054f;      
    float SIG_G_D = 0.0001f;     
    float SIG_GPS_P_NE = 1.00f;  
    float SIG_GPS_P_D = 1.50f;   
    float SIG_GPS_V_NE = 0.05f;  
    float SIG_GPS_V_D = 0.10f;
    float SIG_MAG = 0.10f;

    gpsPosData latestGpsPos{};
    gpsVelData latestGpsVel{};
    imuData latestImu{};
    magData latestMag{};

    bool initialized_ = false;
    bool is_gps_pos_initialized = false;
    bool is_gps_vel_initialized = false;
    bool is_imu_initialized = false;
    bool is_mag_initialized = false;
    
    uint64_t _tprev = 0;
    uint64_t last_gps_correction_time = 0;
    bool has_gps_correction = false;
    uint64_t last_mag_correction_time = 0;
    bool has_mag_correction = false;
    bool use_mag_correction = true;
    bool gps_correction_applied = false;
    bool mag_correction_applied = false;
    
    float phi = 0.0f, theta = 0.0f, psi = 0.0f;
    double vn_ins = 0.0, ve_ins = 0.0, vd_ins = 0.0;
    double lat_ins = 0.0, lon_ins = 0.0, alt_ins = 0.0;
    float Bxc = 0.0f, Byc = 0.0f;
    float abx = 0.0, aby = 0.0, abz = 0.0;
    float gbx = 0.0, gby = 0.0, gbz = 0.0;
    
    Eigen::Matrix<float,15,15> Fs = Eigen::Matrix<float,15,15>::Identity();
    Eigen::Matrix<float,15,15> PHI = Eigen::Matrix<float,15,15>::Zero();
    Eigen::Matrix<float,15,15> P = Eigen::Matrix<float,15,15>::Zero();
    Eigen::Matrix<float,15,12> Gs = Eigen::Matrix<float,15,12>::Zero();
    Eigen::Matrix<float,12,12> Rw = Eigen::Matrix<float,12,12>::Zero();
    Eigen::Matrix<float,15,15> Q = Eigen::Matrix<float,15,15>::Zero();
    Eigen::Matrix<float,3,1> grav = Eigen::Matrix<float,3,1>::Zero();
    Eigen::Matrix<float,3,1> om_ib = Eigen::Matrix<float,3,1>::Zero();
    Eigen::Matrix<float,3,1> f_b = Eigen::Matrix<float,3,1>::Zero();
    Eigen::Matrix<float,3,3> C_N2B = Eigen::Matrix<float,3,3>::Zero();
    Eigen::Matrix<float,3,3> C_B2N = Eigen::Matrix<float,3,3>::Zero();
    Eigen::Matrix<float,3,1> dx = Eigen::Matrix<float,3,1>::Zero();
    Eigen::Matrix<double,3,1> dxd = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> estmimated_ins = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> V_ins = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> lla_ins = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> V_gps = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> lla_gps = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> pos_ecef_ins = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> pos_ecef_gps = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<double,3,1> pos_ned_gps = Eigen::Matrix<double,3,1>::Zero();
    Eigen::Matrix<float,4,1> quat = Eigen::Matrix<float,4,1>::Zero();
    Eigen::Matrix<float,4,1> dq = Eigen::Matrix<float,4,1>::Zero();
    
    Eigen::Matrix<float,7,1> y = Eigen::Matrix<float,7,1>::Zero();
    Eigen::Matrix<float,7,7> R = Eigen::Matrix<float,7,7>::Zero();
    Eigen::Matrix<float,15,1> x = Eigen::Matrix<float,15,1>::Zero();
    Eigen::Matrix<float,15,7> K = Eigen::Matrix<float,15,7>::Zero();
    Eigen::Matrix<float,7,15> H = Eigen::Matrix<float,7,15>::Zero();

    Eigen::Matrix<float,3,3> sk(Eigen::Matrix<float,3,1> w);

    void ekf_init(uint64_t time, double vn,double ve,double vd, double lat,double lon,double alt, float p,float q,float r, float ax,float ay,float az, float hx,float hy, float hz);
    Eigen::Matrix<double,3,1> llarate(Eigen::Matrix<double,3,1> V, Eigen::Matrix<double,3,1> lla);
    Eigen::Matrix<double,3,1> llarate(Eigen::Matrix<double,3,1> V, double lat, double alt);
    Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla);
    Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef, Eigen::Matrix<double,3,1> pos_ref);
    Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q);
    Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q);
    float constrainAngle180(float dta);
    float constrainAngle360(float dta);
    constexpr std::pair<double, double> earthradius(double lat);
    Eigen::Matrix<float,4,1> toQuaternion(float yaw, float pitch, float roll);
    std::tuple<float, float, float> toEulerAngles(Eigen::Matrix<float,4,1> quat);
    
    void updateJacobianMatrix();
    void updateProcessNoiseCovarianceTime(float _dt);
    void updateNoiseMatrix();
    void updateBias(float ax,float ay,float az,float p,float q, float r);
    void update15statesAfterKF();
    void updateCalculatedVsPredicted(float hx, float hy, float hz);
    void ekf_update_internal(uint64_t time, bool applyGpsCorrection);
    void updateINS();
    
    bool isValidImu(const imuData& imu) const;
    bool isValidGpsPos(const gpsPosData& pos) const;
    bool isValidGpsVel(const gpsVelData& vel) const;
    bool isValidMag(const magData& mag) const;
};
