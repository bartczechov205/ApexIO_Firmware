#include "ekfNavINS.h"

namespace {
constexpr uint64_t maxImuIntervalUs = 250000;
constexpr uint64_t maxGpsAgeUs = 500000;
constexpr uint64_t maxMagAgeUs = 500000;

bool elapsedUs(uint64_t now, uint64_t before, uint64_t &elapsed) {
    if (now <= UINT32_MAX && before <= UINT32_MAX) {
        const uint32_t delta = static_cast<uint32_t>(now) - static_cast<uint32_t>(before);
        if (delta < UINT32_MAX / 2U) {
            elapsed = delta;
            return true;
        }
        return false;
    }
    if (now >= before) {
        elapsed = now - before;
        return true;
    }
    return false;
}

bool newerSample(uint64_t now, uint64_t before) {
    uint64_t elapsed = 0;
    return elapsedUs(now, before, elapsed) && elapsed > 0;
}
}

bool ekfNavINS::isValidImu(const imuData& imu) const {
    return std::isfinite(imu.acclX) && std::isfinite(imu.acclY) && std::isfinite(imu.acclZ) &&
           std::isfinite(imu.gyroX) && std::isfinite(imu.gyroY) && std::isfinite(imu.gyroZ);
}

bool ekfNavINS::isValidGpsPos(const gpsPosData& pos) const {
    return std::isfinite(pos.lat) && std::isfinite(pos.lon) && std::isfinite(pos.alt)
        && std::abs(pos.lat) < M_PI / 2.0 - 1e-6
        && std::abs(pos.lon) <= M_PI
        && pos.alt > -EARTH_RADIUS / 2.0;
}

bool ekfNavINS::isValidGpsVel(const gpsVelData& vel) const {
    return std::isfinite(vel.vN) && std::isfinite(vel.vE) && std::isfinite(vel.vD);
}

bool ekfNavINS::isValidMag(const magData& mag) const {
    const double norm2 = double(mag.hX) * mag.hX + double(mag.hY) * mag.hY + double(mag.hZ) * mag.hZ;
    return std::isfinite(norm2) && norm2 > 1e-12;
}

void ekfNavINS::ekf_init(uint64_t time, double vn,double ve,double vd,double lat,double lon,double alt,float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz) {
  gbx = p;
  gby = q;
  gbz = r;
  std::tie(theta,phi,psi) = getPitchRollYaw(ax, ay, az, hx, hy, hz);
  quat = toQuaternion(psi, theta, phi);

  grav(2,0) = G;
  
  H.block(0,0,6,6) = Eigen::Matrix<float,6,6>::Identity();
  
  H.row(6).setZero(); 

  updateNoiseMatrix();
  P.block(0,0,3,3) = powf(P_P_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(3,3,3,3) = powf(P_V_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(6,6,2,2) = 0.25f * powf(P_A_INIT,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  P(8,8) = 0.25f * powf(P_HDG_INIT,2.0f);
  P.block(9,9,3,3) = powf(P_AB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  P.block(12,12,3,3) = powf(P_GB_INIT,2.0f) * Eigen::Matrix<float,3,3>::Identity();
  
  R.block(0,0,2,2) = powf(SIG_GPS_P_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(2,2) = powf(SIG_GPS_P_D,2.0f);
  R.block(3,3,2,2) = powf(SIG_GPS_V_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
  R(5,5) = powf(SIG_GPS_V_D,2.0f);
  R(6,6) = powf(SIG_MAG,2.0f); 

  lat_ins = lat;
  lon_ins = lon;
  alt_ins = alt;
  vn_ins = vn;
  ve_ins = ve;
  vd_ins = vd;
  C_N2B = quat2dcm(quat);
  C_B2N = C_N2B.transpose();
  updateBias(ax, ay, az, p, q, r);
  updateINS();
  _tprev = time;
}

void ekfNavINS::ekf_update(uint64_t time, double vn,double ve,double vd,double lat,double lon,double alt,
                          float p,float q,float r,float ax,float ay,float az,float hx,float hy, float hz,
                          bool applyGpsCorrection) {
  gps_correction_applied = false;
  mag_correction_applied = false;
  const imuData inputImu{time, p, q, r, ax, ay, az};
  const gpsPosData inputPos{time, lat, lon, alt};
  const gpsVelData inputVel{time, vn, ve, vd};
  const magData inputMag{time, hx, hy, hz};
  if (!isValidImu(inputImu)) return;
  if ((!initialized_ || applyGpsCorrection) &&
      (!isValidGpsPos(inputPos) || !isValidGpsVel(inputVel))) return;
  if (!initialized_ && (!isValidMag(inputMag) ||
      std::hypot(double(ax), std::hypot(double(ay), double(az))) < 1e-6)) return;

  if (!initialized_) {
    ekf_init(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz);
    initialized_ = true;
    gps_correction_applied = true;
    mag_correction_applied = true;
  } else {
    uint64_t elapsed = 0;
    if (!elapsedUs(time, _tprev, elapsed) || elapsed == 0) return;
    if (elapsed > maxImuIntervalUs) {
      _tprev = time;
      return;
    }
    const float _dt = static_cast<float>(elapsed) * 1e-6f;

    updateBias(ax, ay, az, p, q, r);
    updateINS();

    dq(0) = 1.0f;
    dq(1) = 0.5f*om_ib(0,0)*_dt;
    dq(2) = 0.5f*om_ib(1,0)*_dt;
    dq(3) = 0.5f*om_ib(2,0)*_dt;
    quat = qmult(quat,dq);
    quat.normalize();

    if (quat(0) < 0) {
      quat = -1.0f*quat;
    }

    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();
    std::tie(phi, theta, psi) = toEulerAngles(quat);

    dx = C_B2N*f_b + grav;
    vn_ins += _dt*dx(0,0);
    ve_ins += _dt*dx(1,0);
    vd_ins += _dt*dx(2,0);

    dxd = llarate(V_ins,lla_ins);
    lat_ins += _dt*dxd(0,0);
    lon_ins += _dt*dxd(1,0);
    alt_ins += _dt*dxd(2,0);

    updateJacobianMatrix();
    updateProcessNoiseCovarianceTime(_dt);

    _tprev = time;

    if (applyGpsCorrection) {
      lla_gps(0,0) = lat;
      lla_gps(1,0) = lon;
      lla_gps(2,0) = alt;
      V_gps(0,0) = vn;
      V_gps(1,0) = ve;
      V_gps(2,0) = vd;

      R.block(0,0,2,2) = powf(SIG_GPS_P_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
      R(2,2) = powf(SIG_GPS_P_D,2.0f);
      R.block(3,3,2,2) = powf(SIG_GPS_V_NE,2.0f) * Eigen::Matrix<float,2,2>::Identity();
      R(5,5) = powf(SIG_GPS_V_D,2.0f);
      R(6,6) = powf(SIG_MAG,2.0f); 

      updateINS();
      updateCalculatedVsPredicted(hx, hy, hz);

      const Eigen::Matrix<float,7,7> S = H*P*H.transpose() + R;
      const Eigen::LDLT<Eigen::Matrix<float,7,7>> solver(S);
      if (solver.info() == Eigen::Success && solver.vectorD().allFinite()
          && (solver.vectorD().array() > 0.0f).all()) {
          K = solver.solve((P*H.transpose()).transpose()).transpose();
          x = K*y;
          if (K.allFinite() && x.allFinite()) {
              const Eigen::Matrix<float,15,15> A = Eigen::Matrix<float,15,15>::Identity()-K*H;
              P = (A*P*A.transpose() + K*R*K.transpose()).eval();
              P = (0.5f*(P+P.transpose())).eval();
              update15statesAfterKF();
              gps_correction_applied = true;
              mag_correction_applied = H.row(6).squaredNorm() > 0.0f;
          }
      }
    }
    updateBias(ax, ay, az, p, q, r);
    updateINS();
  }
}

void ekfNavINS::ekf_update_internal(uint64_t time, bool applyGpsCorrection) {
  ekf_update(time, latestGpsVel.vN, latestGpsVel.vE, latestGpsVel.vD,
                   latestGpsPos.lat, latestGpsPos.lon, latestGpsPos.alt,
                   latestImu.gyroX, latestImu.gyroY, latestImu.gyroZ,
                   latestImu.acclX, latestImu.acclY, latestImu.acclZ,
                   latestMag.hX, latestMag.hY, latestMag.hZ, applyGpsCorrection);
}

bool ekfNavINS::imuDataUpdateEKF(const imuData& imu, ekfState* ekfOut) {
  if (!isValidImu(imu)) return false;
  if (is_imu_initialized && !newerSample(imu.imu_time, latestImu.imu_time)) return false;
  latestImu = imu;
  is_imu_initialized = true;

  if (!(is_gps_pos_initialized && is_gps_vel_initialized && is_mag_initialized)) return false;

  uint64_t gpsAge = 0;
  const bool coherentGps = latestGpsPos.pos_time == latestGpsVel.vel_time
      && elapsedUs(imu.imu_time, latestGpsVel.vel_time, gpsAge) && gpsAge <= maxGpsAgeUs;
  if (!initialized_ && !coherentGps) return false;

  if (initialized_) {
      uint64_t elapsed = 0;
      if (!elapsedUs(imu.imu_time, _tprev, elapsed) || elapsed == 0) return false;
      if (elapsed > maxImuIntervalUs) {
          _tprev = imu.imu_time;
          return false;
      }
  }

  const bool applyGpsCorrection = coherentGps &&
      (!has_gps_correction || latestGpsVel.vel_time != last_gps_correction_time);
  uint64_t magAge = 0;
  use_mag_correction = elapsedUs(imu.imu_time, latestMag.mag_time, magAge)
      && magAge <= maxMagAgeUs &&
      (!has_mag_correction || latestMag.mag_time != last_mag_correction_time);
  if (!initialized_ && !use_mag_correction) {
      use_mag_correction = true;
      return false;
  }
  ekf_update_internal(imu.imu_time, applyGpsCorrection);
  if (!initialized_) {
      use_mag_correction = true;
      return false;
  }
  if (gps_correction_applied) {
      last_gps_correction_time = latestGpsVel.vel_time;
      has_gps_correction = true;
      if (mag_correction_applied) {
          last_mag_correction_time = latestMag.mag_time;
          has_mag_correction = true;
      }
  }
  use_mag_correction = true;

  if (ekfOut) {
      ekfOut->timestamp = imu.imu_time;
      ekfOut->lla = Eigen::Vector3d(lat_ins, lon_ins, alt_ins);
      ekfOut->velNED = Eigen::Vector3d(vn_ins, ve_ins, vd_ins);
      ekfOut->linear = f_b;
      ekfOut->angular = om_ib;
      ekfOut->quat = quat;
      ekfOut->cov = P;
      ekfOut->accl_bias = Eigen::Vector3d(abx, aby, abz);
      ekfOut->gyro_bias = Eigen::Vector3d(gbx, gby, gbz);
  }
  return true;
}

void ekfNavINS::magDataUpdateEKF(const magData& mag) {
  if (isValidMag(mag) && (!is_mag_initialized || newerSample(mag.mag_time, latestMag.mag_time))) {
      latestMag = mag;
      is_mag_initialized = true;
  }
}

void ekfNavINS::gpsPosDataUpdateEKF(const gpsPosData& pos) {
  if (isValidGpsPos(pos) && (!is_gps_pos_initialized || newerSample(pos.pos_time, latestGpsPos.pos_time))) {
      latestGpsPos = pos;
      is_gps_pos_initialized = true;
  }
}

void ekfNavINS::gpsVelDataUpdateEKF(const gpsVelData& vel) {
  if (isValidGpsVel(vel) && (!is_gps_vel_initialized || newerSample(vel.vel_time, latestGpsVel.vel_time))) {
      latestGpsVel = vel;
      is_gps_vel_initialized = true;
  }
}

void ekfNavINS::updateINS() {
  lla_ins(0,0) = lat_ins;
  lla_ins(1,0) = lon_ins;
  lla_ins(2,0) = alt_ins;
  V_ins(0,0) = vn_ins;
  V_ins(1,0) = ve_ins;
  V_ins(2,0) = vd_ins;
}

std::tuple<float,float,float> ekfNavINS::getPitchRollYaw(float ax, float ay, float az, float hx, float hy, float hz) {
  theta = atan2f(ax, hypotf(ay, az));
  phi = atan2f(-ay, -az);
  Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
  Byc = hy*cosf(phi) - hz*sinf(phi);
  psi = -atan2f(Byc, Bxc);
  return std::make_tuple(theta, phi, psi);
}

void ekfNavINS::updateCalculatedVsPredicted(float hx, float hy, float hz) {
  pos_ecef_ins = lla2ecef(lla_ins);
  pos_ecef_gps = lla2ecef(lla_gps);
  pos_ned_gps = ecef2ned(pos_ecef_gps - pos_ecef_ins, lla_ins);
  
  y(0,0) = (float)(pos_ned_gps(0,0));
  y(1,0) = (float)(pos_ned_gps(1,0));
  y(2,0) = (float)(pos_ned_gps(2,0));
  y(3,0) = (float)(V_gps(0,0) - V_ins(0,0));
  y(4,0) = (float)(V_gps(1,0) - V_ins(1,0));
  y(5,0) = (float)(V_gps(2,0) - V_ins(2,0));
  
  float Bxc_mag = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
  float Byc_mag = hy*cosf(phi) - hz*sinf(phi);
  float psi_mag = -atan2f(Byc_mag, Bxc_mag);
  
  H.row(6).setZero();
  y(6,0) = 0.0f;
  const float cosPitch = cosf(theta);
  const float horizontalField = hypotf(Bxc_mag, Byc_mag);
  if (use_mag_correction && std::isfinite(horizontalField) && horizontalField > 1e-6f
      && std::abs(cosPitch) > 1e-3f) {
      H(6,7) = 2.0f*sinf(phi)/cosPitch;
      H(6,8) = 2.0f*cosf(phi)/cosPitch;
      y(6,0) = constrainAngle180(psi_mag - psi);
  }
}

void ekfNavINS::update15statesAfterKF() {
  estmimated_ins = llarate ((x.block(0,0,3,1)).cast<double>(), lat_ins, alt_ins);
  lat_ins += estmimated_ins(0,0);
  lon_ins += estmimated_ins(1,0);
  alt_ins += estmimated_ins(2,0);
  vn_ins += x(3,0);
  ve_ins += x(4,0);
  vd_ins += x(5,0);
  
  dq(0,0) = 1.0f;
  dq(1,0) = x(6,0);
  dq(2,0) = x(7,0);
  dq(3,0) = x(8,0);
  quat = qmult(quat,dq);
  quat.normalize();
  
  std::tie(phi, theta, psi) = toEulerAngles(quat);
  C_N2B = quat2dcm(quat);
  C_B2N = C_N2B.transpose();

  Eigen::Matrix<float,15,15> reset = Eigen::Matrix<float,15,15>::Identity();
  reset.block(6,6,3,3) -= sk(x.block(6,0,3,1));
  P = (reset*P*reset.transpose()).eval();
  P = (0.5f*(P+P.transpose())).eval();
  abx += x(9,0);
  aby += x(10,0);
  abz += x(11,0);
  gbx += x(12,0);
  gby += x(13,0);
  gbz += x(14,0);
}

void ekfNavINS::updateBias(float ax,float ay,float az,float p,float q, float r) {
  f_b(0,0) = ax - abx;
  f_b(1,0) = ay - aby;
  f_b(2,0) = az - abz;
  om_ib(0,0) = p - gbx;
  om_ib(1,0) = q - gby;
  om_ib(2,0) = r - gbz;
}

void ekfNavINS::updateProcessNoiseCovarianceTime(float _dt) {
PHI = Eigen::Matrix<float,15,15>::Identity()+Fs*_dt;
  updateNoiseMatrix();
  Gs.setZero();
  Gs.block(3,0,3,3) = -C_B2N;
  Gs.block(6,3,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  Gs.block(9,6,6,6) = Eigen::Matrix<float,6,6>::Identity();

  const Eigen::Matrix<float,15,15> noise = Gs*Rw*Gs.transpose();
  Q = (0.5f*_dt*(noise + PHI*noise*PHI.transpose())).eval();
  Q = (0.5f*(Q+Q.transpose())).eval();
  P = (PHI*P*PHI.transpose()+Q).eval();
  P = (0.5f*(P+P.transpose())).eval();
}

void ekfNavINS::updateJacobianMatrix() {
  Fs.setZero();
  Fs.block(0,3,3,3) = Eigen::Matrix<float,3,3>::Identity();
  Fs(5,2) = -2.0f*G/EARTH_RADIUS;
  Fs.block(3,6,3,3) = -2.0f*C_B2N*sk(f_b);
  Fs.block(3,9,3,3) = -C_B2N;
  Fs.block(6,6,3,3) = -sk(om_ib);
  Fs.block(6,12,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
  Fs.block(9,9,3,3) = -1.0f/TAU_A*Eigen::Matrix<float,3,3>::Identity();
  Fs.block(12,12,3,3) = -1.0f/TAU_G*Eigen::Matrix<float,3,3>::Identity();
}

Eigen::Matrix<float,3,3> ekfNavINS::sk(Eigen::Matrix<float,3,1> w) {
  Eigen::Matrix<float,3,3> C;
  C(0,0) = 0.0f;    C(0,1) = -w(2,0); C(0,2) = w(1,0);
  C(1,0) = w(2,0);  C(1,1) = 0.0f;    C(1,2) = -w(0,0);
  C(2,0) = -w(1,0); C(2,1) = w(0,0);  C(2,2) = 0.0f;
  return C;
}

constexpr std::pair<double, double> ekfNavINS::earthradius(double lat) {
  double denom = fabs(1.0 - (ECC2 * pow(sin(lat),2.0)));
  double Rew = EARTH_RADIUS / sqrt(denom);
  double Rns = EARTH_RADIUS * (1.0-ECC2) / (denom*sqrt(denom));
  return std::make_pair(Rew, Rns);
}

Eigen::Matrix<double,3,1> ekfNavINS::llarate(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla) {
  double Rew, Rns;
  Eigen::Matrix<double,3,1> lla_dot;
  std::tie(Rew, Rns) = earthradius(lla(0,0));
  lla_dot(0,0) = V(0,0)/(Rns + lla(2,0));
  lla_dot(1,0) = V(1,0)/((Rew + lla(2,0))*cos(lla(0,0)));
  lla_dot(2,0) = -V(2,0);
  return lla_dot;
}

Eigen::Matrix<double,3,1> ekfNavINS::llarate(Eigen::Matrix<double,3,1> V, double lat, double alt) {
  Eigen::Matrix<double,3,1> lla;
  lla(0,0) = lat;
  lla(1,0) = 0.0;
  lla(2,0) = alt;
  return llarate(V, lla);
}

Eigen::Matrix<double,3,1> ekfNavINS::lla2ecef(Eigen::Matrix<double,3,1> lla) {
  double Rew;
  Eigen::Matrix<double,3,1> ecef;
  std::tie(Rew, std::ignore) = earthradius(lla(0,0));
  ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
  ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
  ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
  return ecef;
}

Eigen::Matrix<double,3,1> ekfNavINS::ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
  Eigen::Matrix<double,3,1> ned;
  ned(1,0) = -sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
  ned(0,0) = -sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
  ned(2,0) = -cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
  return ned;
}

Eigen::Matrix<float,3,3> ekfNavINS::quat2dcm(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,3> C_N2B;
  C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
  C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
  C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);

  C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
  C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);

  C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
  C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);

  C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
  C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
  return C_N2B;
}

Eigen::Matrix<float,4,1> ekfNavINS::qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,4,1> r;
  r(0,0) = p(0,0)*q(0,0) - (p(1,0)*q(1,0) + p(2,0)*q(2,0) + p(3,0)*q(3,0));
  r(1,0) = p(0,0)*q(1,0) + q(0,0)*p(1,0) + p(2,0)*q(3,0) - p(3,0)*q(2,0);
  r(2,0) = p(0,0)*q(2,0) + q(0,0)*p(2,0) + p(3,0)*q(1,0) - p(1,0)*q(3,0);
  r(3,0) = p(0,0)*q(3,0) + q(0,0)*p(3,0) + p(1,0)*q(2,0) - p(2,0)*q(1,0);
  return r;
}

float ekfNavINS::constrainAngle180(float dta) {
  if (!std::isfinite(dta)) return dta;
  return std::remainder(dta, static_cast<float>(2.0*M_PI));
}

float ekfNavINS::constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}

Eigen::Matrix<float,4,1> ekfNavINS::toQuaternion(float yaw, float pitch, float roll) {
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    Eigen::Matrix<float,4,1> q;
    q(0) = cr * cp * cy + sr * sp * sy;
    q(1) = sr * cp * cy - cr * sp * sy;
    q(2) = cr * sp * cy + sr * cp * sy;
    q(3) = cr * cp * sy - sr * sp * cy;
    return q;
}

std::tuple<float, float, float> ekfNavINS::toEulerAngles(Eigen::Matrix<float,4,1> quat) {
    float roll, pitch, yaw;
    float sinr_cosp = 2.0f * (quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0));
    float cosr_cosp = 1.0f - 2.0f * (quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0));
    roll = atan2f(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0f * (quat(0,0)*quat(2,0) - quat(1,0)*quat(3,0));
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2.0f, sinp);
    else
        pitch = asinf(sinp);
        
    float siny_cosp = 2.0f * (quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0));
    float cosy_cosp = 1.0f - 2.0f * (quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0));
    yaw = atan2f(siny_cosp, cosy_cosp);
    
    return std::make_tuple(roll, pitch, yaw);
}
void ekfNavINS::updateNoiseMatrix() {
  Rw.setZero();
  Rw.block(0,0,3,3) = powf(SIG_W_A,2.0f)*Eigen::Matrix<float,3,3>::Identity();
  Rw.block(3,3,3,3) = powf(SIG_W_G,2.0f)*Eigen::Matrix<float,3,3>::Identity();
  Rw.block(6,6,3,3) = (2.0f*powf(SIG_A_D,2.0f)/TAU_A)*Eigen::Matrix<float,3,3>::Identity();
  Rw.block(9,9,3,3) = (2.0f*powf(SIG_G_D,2.0f)/TAU_G)*Eigen::Matrix<float,3,3>::Identity();
}
