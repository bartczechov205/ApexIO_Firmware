#pragma once

#include "TelemetryData.h"
#include "AppStateManager.h"
#include "Macro.h"
#include "MathUtils.h"
#include "GeoLib.h"

enum Drag_State
{
    Drag_Waiting,
    Drag_Ready,
    Drag_Measuring,
    Drag_Finished,
};

enum LapTimer_State
{
    LapTimer_Waiting,
    LapTimer_Calibration,
    LapTimer_LapZero,
    LapTimer_Measuring,
    LapTimer_Finished,
};

struct LapTimerPoint
{
    double x;
    double y;
};

struct DeltaLapTimerPoint
{
    uint32_t nr;
    uint32_t lapTime;
};

struct GateLapTimerPoint
{
    uint32_t nr;
    double P1x;
    double P1y;
    double P2x;
    double P2y;
};

class TelemetryEngine
{
private:
    SystemData localSystemData;
    AxisCalibData axisCalibDataBox;
    LiveTeleData liveTeleDataBox;
    DragModeData dragModeDataBox;
    LapTimerData lapTimerDataBox;
    SDData lapTimerSDDataBox;

    AppState previousState = State_Splash;
    double distance_km = 0;
    double lastdistance_km = 0;
    double breakingStartDisM = 0;
    double brakingDistanceM = 0;
    uint32_t nowTime = 0;
    uint32_t lastTime = 0;
    double dt_s;

    double Vm = 0.0;
    double Vkph = 0.0;

    double speedTabelLT[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double speedTabelDM[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double speedTabelLapT[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Drag_State dragState = Drag_Waiting;
    bool isStoped = false;

    bool isReady0100 = false;
    bool isReady60ft = false;
    bool isReady100 = false;
    bool isReady200 = false;
    bool isReady400 = false;
    bool isReadyBrkDis = false;
    bool hasStartPoint = false;

    uint32_t stopTime = 0;
    uint32_t startMeasurementTime = 0;

    LapTimer_State lapTimerState = LapTimer_Waiting;

    uint32_t startTime = 0;

    LapTimerPoint calibP1 = {0, 0};
    LapTimerPoint strLineP1 = {0, 0};
    LapTimerPoint strLineP2 = {0, 0};
    LapTimerPoint previousPosition{0, 0};

    double calibL = 0.0;
    double uX = 0.0;
    double uY = 0.0;

    bool hasPreviousPosition = false;
    double totalMoveL = 0.0;
    bool lineArmed = false;
    bool okPressedReceived = false;

    bool isFirstSampleLogged = false;
    uint32_t lastLogTimeMs = 0;

    LapTimerPoint gatePosition{0, 0};
    bool hasGatePosition = false;

    std::vector<DeltaLapTimerPoint> currentRoute;
    std::vector<DeltaLapTimerPoint> bestRoute;
    std::vector<GateLapTimerPoint> gates;

    uint32_t previousTimeSample = 0;
    int32_t currentGateNumber = 0;
    uint32_t crossingTime = 0;

    MathUtils mathUtils;
    GeoLib geoLib;

public:
    void updateTelemetryEngine(SystemData &systemData);
    void processDataForState(AppStateManager &appStateManager);
    void updateAxisCalib();
    void updateLiveTele();
    void updateDragMode();
    void dragModeRestart();
    void updateLapTimer();
    void lapTimerRestart();
    void lapTimerCrossingGates(double t, double check_t, uint32_t currentTimeSample, double dx, double dy);
    void updateLapDelta(double Qx, double Qy, uint32_t currentTimeSample);
    void proccesDataforSD();
};