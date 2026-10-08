#pragma once

#include <Arduino.h>
#include "ekfNavINS.h"

struct gpsExtraData
{
  uint8_t siv;
  uint8_t hr;
  uint8_t min;
  uint8_t sec;
  uint16_t ms;
  int32_t ns;
  int32_t alt_mm;
  int32_t hdg;
};

struct ekfData
{
  double vN;
  double vE;
  double vD;
  double lat_rad;
  double lon_rad;
  double alt_m;
  float hdg_rad;
};

struct SystemData
{
  ekfData ekfOutputData;
  imuData rawImuData;
  gpsExtraData rawGpsExtraData;
};

struct AxisCalibData
{
  float gX = 0.0f;
  float gY = 0.0f;
};

struct LiveTeleData
{
  float speedKmh;
  float gX;
  float gY;
  double dis;
  int32_t alt_m;
  String hdg;
};

enum LapStatus
{
  Waiting,
  Calibration,
  LapZero,
  Measuring
};

enum DragStatus
{
  DragStop,
  DragReady,
  DragMeasuring,
  DragFinished,
};

struct DragModeData
{
  float speedKmh;
  float gX;
  double dis;
  double t60ft;
  double t100m;
  double t200m;
  double t400m;
  double t0100;
  double timer;
  bool isMeasuring;
  bool r60ft;
  bool r100m;
  bool r200m;
  bool r400m;
  bool r0100;
  double brakingDis;
  double Vmax;
  DragStatus status = DragStatus::DragStop;
};

struct LapTimerData
{
  double speedKmh;
  double bestTimeS;
  double lastTimeS;
  double counter;
  double delta;
  double gY;
  double gX;
  bool isMeasuring;
  bool isFirstLapComplet;
  bool isLapValid;
  LapStatus status = LapStatus::Waiting;
};

struct SDData
{
  uint8_t siv;
  uint32_t timeMs;
  double lat;
  double lon;
  float velocityKmh;
  float headingDeg;
  float heightM;
  float verticalVelocity;
  float samplePeriodS;
  int8_t hr;
  int8_t min;
  bool isMeasuring;
};
