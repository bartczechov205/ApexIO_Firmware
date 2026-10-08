#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "TelemetryData.h"
#include "AppStateManager.h"
#include "SDManager.h"
#include "AppResources.h"


extern TFT_eSPI tft;

enum SettingsGateSetupState
{
    StartLineWidthSel,
    StartLineWidth,
    GateWidthSel,
    GateWidth,
    DeltaMaxSel,
    DeltaMax,
};

struct DisplayResources
{
    AppState lastState = State_Splash;

    AxisCalibData axisCalibDataReceived;
    LiveTeleData liveTeleDataReceived;
    DragModeData dragModeDataReceived;
    LapTimerData lapTimerDataReceived;

    bool validFix = false;
    bool systemReady = false;

    uint32_t startTime;
    uint32_t lastStatusUpdate = 0;
    uint32_t lastAxisCalibUpdate = 0;
};

class TFTManager
{

float voltage = 0.0;

public:

void updateDisplay(AppResources &resources,AppStateManager &appStateManager,DisplayResources &displayResources);
void begin();
void drawBattery();
void drawSplashScreen();
void drawSplashInfo(bool validFix);

void drawSDStatus();
void drawSivStatus(uint8_t siv);
void drawStatus(uint8_t siv);


void drawLiveTele(LiveTeleData &liveTeleDataReceived, uint8_t siv);
void drawLiveTeleSel(uint8_t siv);
void backgroundLiveTele(uint8_t siv);

void drawDragMode(DragModeData &DragModeDataReceived, uint8_t siv);
void drawDragModeSel(uint8_t siv);
void backgroundDragMode(uint8_t siv);

void drawLapTimer(LapTimerData &lapTimerDataReceived, uint8_t siv);
void backgroundLapTimer(uint8_t siv);
void drawLapTimerSel(uint8_t siv);

void drawSettingsSel(uint8_t siv);
void drawSettingsAxisCalibSel(uint8_t siv);
void drawSettingsAxisCalib(AxisCalibData &axisCalibDataReceived, uint8_t siv);
void drawSettingsUTCOffsetSel(uint8_t siv);
void drawSettingsGateSetupSel(uint8_t siv);
void drawSettingsAboutSel(uint8_t siv);

void drawSettingsUTCOffset(uint8_t siv);
void drawSettingsGateSetup(AppState state, uint8_t siv);
void drawSettingsAbout(uint8_t siv);

SettingsGateSetupState gateSetupSate; 

};