#pragma once

#include <Arduino.h>
#include "Macro.h"

enum AppState
{
    State_Splash,
    State_LiveTeleSel,
    State_DragModeSel,
    State_LapTimerSel,
    State_SettingsSel,
    State_LiveTele,
    State_DragMode,
    State_LapTimer,
    State_Settings_Axis_CalibSel,
    State_Settings_Axis_Calib,
    State_Settings_UTC_OFFSETSel,
    State_Settings_UTC_OFFSET,
    State_Settings_GATE_SETUPSel,
    State_Settings_ABOUTSel,
    State_Settings_ABOUT,
    State_StartLineWidthSel,
    State_GateWidthSel,
    State_DeltaMaxSel,
    State_StartLineWidth,
    State_GateWidth,
    State_DeltaMax,
};

struct Button
{
    int8_t pin;
    bool lastPinState;
    uint32_t lastButtonPressTime;
};

class AppStateManager
{
private:
    Button btn_up{BTN_UP, HIGH, 0};
    Button btn_down{BTN_DOWN, HIGH, 0};
    Button btn_ok{BTN_OK, HIGH, 0};
    Button btn_back{BTN_BACK, HIGH, 0};

    bool okPressed = false;
    bool wasPressed = false;

    int8_t debounceDelay = 50;

    AppState currentState;

public:
    void begin();
    void updateState(bool validFix, bool canContinue);
    bool isDragOkPressed();
    AppState getState();
};