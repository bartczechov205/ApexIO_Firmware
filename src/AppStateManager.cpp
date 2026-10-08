#include "AppStateManager.h"
#include "AppResources.h"

extern AppResources resources;

void AppStateManager::begin()
{
    currentState = State_Splash;
    okPressed = false;
    wasPressed = false;

    Serial.println("Splash screen, press OK");
}

void AppStateManager::updateState(bool validFix, bool canContinue)
{
    bool upState = digitalRead(btn_up.pin);
    bool downState = digitalRead(btn_down.pin);
    bool okState = digitalRead(btn_ok.pin);
    bool backState = digitalRead(btn_back.pin);

    bool upClick = false;
    bool downClick = false;
    bool okClick = false;
    bool backClick = false;

    if (upState == LOW && btn_up.lastPinState == HIGH)
    {
        if (millis() - btn_up.lastButtonPressTime > debounceDelay)
        {
            upClick = true;
            btn_up.lastButtonPressTime = millis();
        }
    }

    if (downState == LOW && btn_down.lastPinState == HIGH)
    {
        if (millis() - btn_down.lastButtonPressTime > debounceDelay)
        {
            downClick = true;
            btn_down.lastButtonPressTime = millis();
        }
    }

    if (okState == LOW && btn_ok.lastPinState == HIGH)
    {
        if (millis() - btn_ok.lastButtonPressTime > debounceDelay)
        {
            okClick = true;
            btn_ok.lastButtonPressTime = millis();
        }
    }

    if (backState == LOW && btn_back.lastPinState == HIGH)
    {
        if (millis() - btn_back.lastButtonPressTime > debounceDelay)
        {
            backClick = true;
            btn_back.lastButtonPressTime = millis();
        }
    }

    switch (currentState)
    {
    case State_Splash:
        if (validFix && canContinue && okClick)
        {
            currentState = State_LiveTeleSel;
        }
        break;

    case State_LiveTeleSel:
        if (okClick)
        {
            currentState = State_LiveTele;
        }
        else if (downClick)
        {
            currentState = State_DragModeSel;
        }
        else if (upClick)
        {
            currentState = State_SettingsSel;
        }
        break;

    case State_DragModeSel:
        if (okClick)
        {
            currentState = State_DragMode;
        }
        else if (downClick)
        {
            currentState = State_LapTimerSel;
        }
        else if (upClick)
        {
            currentState = State_LiveTeleSel;
        }
        break;

    case State_LapTimerSel:
        if (okClick)
        {
            currentState = State_LapTimer;
        }
        else if (downClick)
        {
            currentState = State_SettingsSel;
        }
        else if (upClick)
        {
            currentState = State_DragModeSel;
        }
        break;

    case State_SettingsSel:
        if (okClick)
        {
            currentState = State_Settings_Axis_CalibSel;
        }
        else if (downClick)
        {
            currentState = State_LiveTeleSel;
        }
        else if (upClick)
        {
            currentState = State_LapTimerSel;
        }
        break;

    case State_LiveTele:
        if (backClick)
        {
            currentState = State_LiveTeleSel;
        }
        break;

    case State_DragMode:
        if (backClick)
        {
            currentState = State_DragModeSel;
        }
        else if (okClick)
        {
            okPressed = true;
        }
        break;

    case State_LapTimer:
        if (backClick)
        {
            currentState = State_LapTimerSel;
        }
        else if (okClick)
        {
            okPressed = true;
        }
        break;

    case State_Settings_Axis_CalibSel:
        if (backClick)
        {
            currentState = State_SettingsSel;
        }
        else if (okClick)
        {
            currentState = State_Settings_Axis_Calib;
        }
        else if (downClick)
        {
            currentState = State_Settings_UTC_OFFSETSel;
        }
        else if (upClick)
        {
            currentState = State_Settings_ABOUTSel;
        }
        break;

    case State_Settings_UTC_OFFSETSel:
        if (backClick)
        {
            currentState = State_SettingsSel;
        }
        else if (okClick)
        {
            currentState = State_Settings_UTC_OFFSET;
        }
        else if (downClick)
        {
            currentState = State_Settings_GATE_SETUPSel;
        }
        else if (upClick)
        {
            currentState = State_Settings_Axis_CalibSel;
        }
        break;

    case State_Settings_GATE_SETUPSel:
        if (backClick)
        {
            currentState = State_SettingsSel;
        }
        else if (okClick)
        {
            currentState = State_StartLineWidthSel;
        }
        else if (downClick)
        {
            currentState = State_Settings_ABOUTSel;
        }
        else if (upClick)
        {
            currentState = State_Settings_UTC_OFFSETSel;
        }
        break;

    case State_Settings_ABOUTSel:
        if (backClick)
        {
            currentState = State_SettingsSel;
        }
        else if (okClick)
        {
            currentState = State_Settings_ABOUT;
        }
        else if (downClick)
        {
            currentState = State_Settings_Axis_CalibSel;
        }
        else if (upClick)
        {
            currentState = State_Settings_GATE_SETUPSel;
        }
        break;

    case State_Settings_Axis_Calib:
        if (backClick)
        {
            currentState = State_Settings_Axis_CalibSel;
        }
        break;

    case State_Settings_UTC_OFFSET:
        if (backClick)
        {
            currentState = State_Settings_UTC_OFFSETSel;
        }
        else if (upClick)
        {
            if (resources.utcOffset < 12)
            {
                resources.utcOffset++;
            }
        }
        else if (downClick)
        {
            if (resources.utcOffset > -12)
            {
                resources.utcOffset--;
            }
        }
        break;

    case State_StartLineWidthSel:
        if (backClick)
        {
            currentState = State_Settings_GATE_SETUPSel;
        }
        else if (okClick)
        {
            currentState = State_StartLineWidth;
        }
        else if (downClick)
        {
            currentState = State_GateWidthSel;
        }
        else if (upClick)
        {
            currentState = State_DeltaMaxSel;
        }
        break;

    case State_GateWidthSel:
        if (backClick)
        {
            currentState = State_Settings_GATE_SETUPSel;
        }
        else if (okClick)
        {
            currentState = State_GateWidth;
        }
        else if (downClick)
        {
            currentState = State_DeltaMaxSel;
        }
        else if (upClick)
        {
            currentState = State_StartLineWidthSel;
        }
        break;

    case State_DeltaMaxSel:
        if (backClick)
        {
            currentState = State_Settings_GATE_SETUPSel;
        }
        else if (okClick)
        {
            currentState = State_DeltaMax;
        }
        else if (downClick)
        {
            currentState = State_StartLineWidthSel;
        }
        else if (upClick)
        {
            currentState = State_GateWidthSel;
        }
        break;

    case State_StartLineWidth:
        if (backClick)
        {
            currentState = State_StartLineWidthSel;
        }
        else if (upClick)
        {
            if (resources.startLineWidth < 25)
            {
                resources.startLineWidth++;
            }
        }
        else if (downClick)
        {
            if (resources.startLineWidth > 5)
            {
                resources.startLineWidth--;
            }
        }
        break;

    case State_GateWidth:
        if (backClick)
        {
            currentState = State_GateWidthSel;
        }
        else if (upClick)
        {
            if (resources.gateWidth < 25)
            {
                resources.gateWidth++;
            }
        }
        else if (downClick)
        {
            if (resources.gateWidth > 5)
            {
                resources.gateWidth--;
            }
        }
        break;

    case State_DeltaMax:
        if (backClick)
        {
            currentState = State_DeltaMaxSel;
        }
        else if (upClick)
        {
            if (resources.deltaMax < 10.0f)
            {
                resources.deltaMax += 0.5f;
            }
        }
        else if (downClick)
        {
            if (resources.deltaMax > 0.5f)
            {
                resources.deltaMax -= 0.5f;
            }
        }
        break;

    case State_Settings_ABOUT:
        if (backClick)
        {
            currentState = State_Settings_ABOUTSel;
        }
        break;
    }

    btn_up.lastPinState = upState;
    btn_down.lastPinState = downState;
    btn_ok.lastPinState = okState;
    btn_back.lastPinState = backState;
}

AppState AppStateManager::getState()
{
    return currentState;
}

bool AppStateManager::isDragOkPressed()
{
    wasPressed = okPressed;
    okPressed = false;

    return wasPressed;
}