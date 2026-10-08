#include "TFTManager.h"
#include "Graphics.h"
#include "Macro.h"
#include <Fonts/Custom/Orbitron_Light6.h>
#include <Adafruit_INA219.h>
#include <math.h>
#include "AppResources.h"

extern AppResources resources;
extern SDManager sdManager;

TFT_eSPI tft;
TFT_eSprite spr(&tft);

Adafruit_INA219 ina219;

bool sprReady = false;
bool ina219Ready = false;

void TFTManager::updateDisplay(AppResources &resources,AppStateManager &appStateManager,DisplayResources &displayResources)
{
    AppState currentState = appStateManager.getState();
    bool changeState = (currentState != displayResources.lastState);
    int8_t siv = 0;

    if (xSemaphoreTake(resources.systemStateMutex, portMAX_DELAY) == pdTRUE)
    {
        resources.systemReady =
            (uint32_t)(millis() - displayResources.startTime) >= 5000;

        displayResources.validFix = resources.validFix;
        displayResources.systemReady = resources.systemReady;
        siv = resources.systemData.rawGpsExtraData.siv;

        xSemaphoreGive(resources.systemStateMutex);
    }

    if (currentState == State_Splash)
    {
        if (displayResources.systemReady)
        {
            drawSplashInfo(displayResources.validFix);
        }
    }
    else
    {
        switch (currentState)
        {
        case State_LiveTeleSel:
            if (changeState)
            {
                drawLiveTeleSel(siv);
            }
            break;

        case State_LiveTele:
            if (changeState)
            {
                backgroundLiveTele(siv);
            }

            if (xQueueReceive(resources.liveTelemetryQueue, &displayResources.liveTeleDataReceived, 0) == pdTRUE)
            {
                drawLiveTele(displayResources.liveTeleDataReceived, siv);
            }
            break;

        case State_DragModeSel:
            if (changeState)
            {
                drawDragModeSel(siv);
            }
            break;

        case State_DragMode:
            if (changeState)
            {
                backgroundDragMode(siv);
            }

            if (xQueueReceive(resources.dragModeQueue, &displayResources.dragModeDataReceived, 0) == pdTRUE)
            {
                drawDragMode(displayResources.dragModeDataReceived, siv);
            }
            break;

        case State_LapTimerSel:
            if (changeState)
            {
                drawLapTimerSel(siv);
            }
            break;

        case State_LapTimer:
            if (changeState)
            {
                backgroundLapTimer(siv);
            }

            if (xQueueReceive(resources.lapTimerQueue, &displayResources.lapTimerDataReceived, 0) == pdTRUE)
            {
                drawLapTimer(displayResources.lapTimerDataReceived, siv);
            }
            break;

        case State_SettingsSel:
            if (changeState)
            {
                drawSettingsSel(siv);
            }
            break;

        case State_Settings_Axis_CalibSel:
            if (changeState)
            {
                drawSettingsAxisCalibSel(siv);
            }
            break;

        case State_Settings_Axis_Calib:
            xQueueReceive(resources.axisCalibQueue, &displayResources.axisCalibDataReceived, 0);

            if (changeState ||
                millis() - displayResources.lastAxisCalibUpdate >= 50)
            {
                drawSettingsAxisCalib(
                    displayResources.axisCalibDataReceived, siv);

                displayResources.lastAxisCalibUpdate = millis();
            }
            break;

        case State_Settings_UTC_OFFSETSel:
            drawSettingsUTCOffsetSel(siv);
            break;

        case State_Settings_UTC_OFFSET:
            drawSettingsUTCOffset(siv);
            break;

        case State_Settings_GATE_SETUPSel:
            if (changeState)
            {
                drawSettingsGateSetupSel(siv);
            }
            break;

        case State_StartLineWidthSel:
            drawSettingsGateSetup(currentState, siv);
            break;

        case State_GateWidthSel:
            drawSettingsGateSetup(currentState, siv);
            break;

        case State_DeltaMaxSel:
            drawSettingsGateSetup(currentState, siv);
            break;

        case State_StartLineWidth:
            drawSettingsGateSetup(currentState, siv);
            break;

        case State_GateWidth:
            drawSettingsGateSetup(currentState, siv);
            break;

        case State_DeltaMax:
            drawSettingsGateSetup(currentState, siv);
            break;

        case State_Settings_ABOUTSel:
            if (changeState)
            {
                drawSettingsAboutSel(siv);
            }
            break;

        case State_Settings_ABOUT:
            if (changeState)
            {
                drawSettingsAbout(siv);
            }
            break;

        default:
            break;
        }

        if (millis() - displayResources.lastStatusUpdate >= 250)
        {
            drawStatus(siv);
            displayResources.lastStatusUpdate = millis();
        }
    }

    displayResources.lastState = currentState;
}

void TFTManager::backgroundLiveTele(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, backgroundLT);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::backgroundDragMode(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, backgroundDM);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::backgroundLapTimer(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, backgroundLPT);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawLiveTeleSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, LTSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawSplashScreen()
{
    if (!sprReady)
    {
        return;
    }

    spr.pushImage(0, 0, 320, 240, SplashScreen);
    spr.pushSprite(0, 0);

    digitalWrite(TFT_BL, LOW);
}

void TFTManager::drawLiveTele(LiveTeleData &liveTeleDataReceived, uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, backgroundLT);

    spr.setTextDatum(MC_DATUM);

    spr.setFreeFont(&Orbitron_Light_32);
    spr.setTextSize(1);

    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.setTextPadding(spr.textWidth(" 999.9"));

    char speedStr[10];
    snprintf(speedStr, sizeof(speedStr), "%5.1f", liveTeleDataReceived.speedKmh);
    spr.drawString(speedStr, 160, 60);

    spr.setFreeFont(&Orbitron_Light_24);
    spr.setTextSize(1);
    spr.setTextColor(TFT_RED, TFT_BLACK);
    spr.setTextPadding(spr.textWidth("+0.00"));

    char gxStr[10];
    snprintf(gxStr, sizeof(gxStr), "%+4.1f", liveTeleDataReceived.gX);
    spr.drawString(gxStr, 75, 125);

    spr.setTextColor(TFT_CYAN, TFT_BLACK);

    char gyStr[10];
    snprintf(gyStr, sizeof(gyStr), "%+4.1f", liveTeleDataReceived.gY);
    spr.drawString(gyStr, 230, 125);

    spr.setTextColor(TFT_GREEN, TFT_BLACK);
    spr.setTextPadding(spr.textWidth("152.0"));

    char customStr[10];
    snprintf(customStr, sizeof(customStr), "%.1f", liveTeleDataReceived.dis);
    spr.drawString(customStr, 155, 212);

    spr.setTextColor(TFT_BLUE, TFT_BLACK);
    spr.setTextPadding(spr.textWidth("NW"));
    spr.drawString(liveTeleDataReceived.hdg, 55, 212);

    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.setTextPadding(spr.textWidth("152.0"));

    snprintf(customStr, sizeof(customStr), "%ld", liveTeleDataReceived.alt_m);
    spr.drawString(customStr, 260, 212);

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}

void TFTManager::drawDragMode(DragModeData &dragModeDataReceived, uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, backgroundDM);

    spr.setTextDatum(MC_DATUM);
    spr.setFreeFont(&Orbitron_Light_32);
    spr.setTextSize(1);

    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.setTextPadding(spr.textWidth(" 999.9"));

    char speedStr[10];
    snprintf(speedStr, sizeof(speedStr), "%5.1f", dragModeDataReceived.speedKmh);
    spr.drawString(speedStr, 160, 60);

    spr.setFreeFont(&orbitron_light6pt7b);
    spr.setTextSize(1);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.setTextPadding(spr.textWidth("99999"));

    char timerStr[12];

    const bool hasResults = 
    dragModeDataReceived.r60ft ||
    dragModeDataReceived.r100m ||
    dragModeDataReceived.r200m ||
    dragModeDataReceived.r400m ||
    dragModeDataReceived.r0100 ||
    dragModeDataReceived.brakingDis != 0.0;

    if (!dragModeDataReceived.isMeasuring && !hasResults)
    {
        const char *statusText = "STOP";

        switch (dragModeDataReceived.status)
        {
        case DragStatus::DragStop:
            statusText = "STOP";
            break;
        case DragStatus::DragReady:
            statusText = "READY";
            break;
        case DragStatus::DragMeasuring:
            statusText = "MEASURING";
            break;
        case DragStatus::DragFinished:
            statusText = "FINISHED";
            break;
        }

        spr.drawString(statusText, 135, 116);
    }
    else
    {
        double result;

        if (dragModeDataReceived.r60ft)
        {
            result = dragModeDataReceived.t60ft;
        }
        else
        {
            result = dragModeDataReceived.timer;
        }
        snprintf(timerStr, sizeof(timerStr), "%6.3f", result);
        spr.drawString(timerStr, 135, 116);

        if (dragModeDataReceived.r60ft)
        {
            if (dragModeDataReceived.r100m)
            {
                result = dragModeDataReceived.t100m;
            }
            else
            {
                result = dragModeDataReceived.timer;
            }
            snprintf(timerStr, sizeof(timerStr), "%6.3f", result);
            spr.drawString(timerStr, 135, 140);
        }

        if (dragModeDataReceived.r100m)
        {
            if (dragModeDataReceived.r200m)
            {
                result = dragModeDataReceived.t200m;
            }
            else
            {
                result = dragModeDataReceived.timer;
            }
            snprintf(timerStr, sizeof(timerStr), "%6.3f", result);
            spr.drawString(timerStr, 135, 164);
        }

        if (dragModeDataReceived.r200m)
        {
            if (dragModeDataReceived.r400m)
            {
                result = dragModeDataReceived.t400m;
            }
            else
            {
                result = dragModeDataReceived.timer;
            }
            snprintf(timerStr, sizeof(timerStr), "%6.3f", result);
            spr.drawString(timerStr, 135, 188);
        }
    }

    if (dragModeDataReceived.t0100)
    {
        snprintf(timerStr, sizeof(timerStr), "%6.3f", dragModeDataReceived.t0100);
        spr.drawString(timerStr, 135, 212);
    }

    if (dragModeDataReceived.Vmax != 0.0)
    {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        snprintf(timerStr, sizeof(timerStr), "%6.2f", dragModeDataReceived.Vmax);
        spr.drawString(timerStr, 258, 165);
    }

    if (dragModeDataReceived.brakingDis != 0.0)
    {
        spr.setTextColor(TFT_GREEN, TFT_BLACK);
        snprintf(timerStr, sizeof(timerStr), "%6.1f", dragModeDataReceived.brakingDis);
        spr.drawString(timerStr, 250, 120);
    }

    spr.setFreeFont(&Roboto_Thin_24);
    spr.setTextColor(TFT_CYAN, TFT_BLACK);
    snprintf(timerStr, sizeof(timerStr), "%+4.1f", dragModeDataReceived.gX);
    spr.drawString(timerStr, 245, 211);

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}

void TFTManager::begin()
{

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    sprReady = spr.createSprite(320, 240) != nullptr;

    if (sprReady)
    {
        spr.fillSprite(TFT_BLACK);
    }

    digitalWrite(TFT_BL, HIGH);
}

void TFTManager::drawDragModeSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, DMSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

static void formatLapTime(double seconds, char *buf, size_t bufSize)
{
    if (seconds < 0)
    {
        seconds = 0;
    }

    long totalMs = lround(seconds * 1000.0);
    int minutes = totalMs / 60000;
    int secs = (totalMs / 1000) % 60;
    int millis = totalMs % 1000;

    snprintf(buf, bufSize, "%d:%02d:%03d", minutes, secs, millis);
}

void TFTManager::drawLapTimerSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, LPTSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawLapTimer(
    LapTimerData &lapTimerDataReceived,
    uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, backgroundLPT);

    spr.setTextDatum(MC_DATUM);
    spr.setFreeFont(&Orbitron_Light_32);
    spr.setTextSize(1);

    spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    spr.setTextPadding(0);

    char speedStr[10];
    snprintf(
        speedStr,
        sizeof(speedStr),
        "%5.1f",
        lapTimerDataReceived.speedKmh);

    spr.drawString(speedStr, 160, 60);

    char lapTimeStr[16];

    spr.setFreeFont(&Roboto_Thin_24);

    spr.setTextColor(TFT_MAGENTA, TFT_BLACK);

    if (lapTimerDataReceived.isMeasuring)
    {
        formatLapTime(
            lapTimerDataReceived.bestTimeS,
            lapTimeStr,
            sizeof(lapTimeStr));

        spr.drawString(lapTimeStr, 75, 125);
    }
    else
    {
        spr.drawString("-----", 75, 125);
    }

    spr.setTextColor(TFT_CYAN, TFT_BLACK);

    if (lapTimerDataReceived.isMeasuring)
    {
        formatLapTime(
            lapTimerDataReceived.lastTimeS,
            lapTimeStr,
            sizeof(lapTimeStr));

        spr.drawString(lapTimeStr, 230, 125);
    }
    else
    {
        spr.drawString("-----", 230, 125);
    }

    if (lapTimerDataReceived.isMeasuring)
    {
        spr.setFreeFont(&Roboto_Thin_24);

        if (lapTimerDataReceived.isLapValid)
        {
            spr.setTextColor(TFT_WHITE, TFT_BLACK);

            formatLapTime(
                lapTimerDataReceived.counter,
                lapTimeStr,
                sizeof(lapTimeStr));

            spr.drawString(lapTimeStr, 75, 165);

            int maxBarWidth = 148;
            int leftBarEnd = 157;
            int rightBarStart = 159;

            int barWidth = lround(
                fabs(lapTimerDataReceived.delta) /
                resources.deltaMax * maxBarWidth);

            barWidth = min(barWidth, maxBarWidth);

            if (lapTimerDataReceived.delta > 0.0)
            {
                spr.fillRect(leftBarEnd - barWidth,183,barWidth,11,TFT_RED);

                spr.fillRect(
                    rightBarStart,
                    183,
                    barWidth,
                    11,
                    TFT_RED);

                spr.setTextColor(TFT_RED, TFT_BLACK);
            }
            else if (lapTimerDataReceived.delta < 0.0)
            {
                spr.fillRect(
                    leftBarEnd - barWidth,
                    183,
                    barWidth,
                    11,
                    TFT_GREEN);

                spr.fillRect(
                    rightBarStart,
                    183,
                    barWidth,
                    11,
                    TFT_GREEN);

                spr.setTextColor(TFT_GREEN, TFT_BLACK);
            }
            else
            {
                spr.setTextColor(TFT_WHITE, TFT_BLACK);
            }

            snprintf(
                lapTimeStr,
                sizeof(lapTimeStr),
                "%+.2f",
                lapTimerDataReceived.delta);

            spr.drawString(lapTimeStr, 250, 163);
        }
        else
        {
            spr.setTextColor(TFT_RED, TFT_BLACK);
            spr.drawString("Lap is not valid", 160, 90);
        }
    }
    else
    {
        spr.setFreeFont(&orbitron_light6pt7b);
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);

        switch (lapTimerDataReceived.status)
        {
        case LapStatus::Waiting:
            spr.drawString("STOP AND PRESS OK", 160, 165);
            break;

        case LapStatus::Calibration:
            spr.drawString(
                "MOVE FORWARD TO SET START LINE",
                160,
                165);
            break;

        case LapStatus::LapZero:
            spr.drawString("LINE SET, COMPLETE THE LAP", 160, 165);
            break;

        case LapStatus::Measuring:
            break;
        }
    }

    spr.setFreeFont(&Roboto_Thin_24);
    spr.setTextColor(TFT_WHITE, TFT_BLACK);
    spr.setTextPadding(0);

    char gxStr[16];
    snprintf(
        gxStr,
        sizeof(gxStr),
        "%+4.1f",
        lapTimerDataReceived.gX);

    spr.drawString(gxStr, 90, 213);

    char gyStr[16];
    snprintf(
        gyStr,
        sizeof(gyStr),
        "%+4.1f",
        lapTimerDataReceived.gY);

    spr.drawString(gyStr, 250, 213);

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}

void TFTManager::drawBattery()
{
    if (!ina219Ready)
    {
        ina219Ready = ina219.begin(&Wire);
        if (!ina219Ready)
        {
            Serial.println("INA219 sensor not found. Check the connections!");
            return;
        }
    }

    voltage = ina219.getBusVoltage_V();

    if (voltage >= 3.95)
    {
        int8_t batX = 23;
        for (int8_t i = 0; i < 4; i++)
        {
            spr.fillRect(batX, 12, 6, 14, TFT_LIGHTGREY);
            batX += 8;
        }
    }
    else if (voltage >= 3.80 && voltage < 3.95)
    {
        int8_t batX = 23;
        for (int8_t i = 0; i < 3; i++)
        {
            spr.fillRect(batX, 12, 6, 14, TFT_LIGHTGREY);
            batX += 8;
        }
    }
    else if (voltage >= 3.65 && voltage < 3.80)
    {
        int8_t batX = 23;
        for (int8_t i = 0; i < 2; i++)
        {
            spr.fillRect(batX, 12, 6, 14, TFT_LIGHTGREY);
            batX += 8;
        }
    }
    else if (voltage >= 3.40 && voltage < 3.65)
    {
        int8_t batX = 23;
        for (int8_t i = 0; i < 1; i++)
        {
            spr.fillRect(batX, 12, 6, 14, TFT_LIGHTGREY);
            batX += 8;
        }
    }
    else
    {
        spr.pushImage(0, 0, 320, 38, LowBattery);
    }
}

void TFTManager::drawSplashInfo(bool validFix)
{

    spr.setFreeFont(&orbitron_light6pt7b);

    if (!validFix)
    {
        spr.fillRect(70, 200, spr.width() - 70, 35, TFT_BLACK);
        spr.drawString("Please wait for GPS fix", 75, 210);
    }
    else
    {
        spr.fillRect(70, 200, spr.width() - 70, 35, TFT_BLACK);
        spr.drawString("Press OK to continue", 85, 210);
    }

    spr.pushSprite(0, 0);
}

void TFTManager::drawSDStatus()
{
    if (sdManager.isCardPresent())
    {
        spr.fillCircle(292, 18, 6, TFT_GREEN);
    }
    else
    {
        spr.fillCircle(292, 18, 6, TFT_RED);
    }
}

void TFTManager::drawSivStatus(uint8_t siv)
{
    if (siv < 10)
    {
        spr.fillCircle(242, 18, 6, TFT_RED);
    }
    else if (siv >= 10 && siv < 15)
    {
        spr.fillCircle(242, 18, 6, TFT_YELLOW);
    }
    else
    {
        spr.fillCircle(242, 18, 6, TFT_GREEN);
    }
}

void TFTManager::drawStatus(uint8_t siv)
{
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}
void TFTManager::drawSettingsSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, STGSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsAxisCalibSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, STGAXSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsAxisCalib(AxisCalibData &axisCalibDataReceived, uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, AXCalib);

    int centerX = 159;
    int centerY = 119;
    int circleRadius = 61;
    int dotRadius = 5;

    float maxG = 0.50f;
    float maxDistance = circleRadius - dotRadius - 2;

    float gX = axisCalibDataReceived.gX;
    float gY = axisCalibDataReceived.gY;

    spr.setTextDatum(MC_DATUM);
    spr.setFreeFont(&orbitron_light6pt7b);
    spr.setTextSize(1);
    spr.setTextPadding(0);
    spr.setTextColor(TFT_YELLOW, TFT_BLACK);

    if (isfinite(gX) && isfinite(gY))
    {
        float dotX = -gX / maxG * circleRadius;
        float dotY = gY / maxG * circleRadius;

        float dotDistance = sqrtf(dotX * dotX + dotY * dotY);

        if (dotDistance > maxDistance)
        {
            dotX = dotX * maxDistance / dotDistance;
            dotY = dotY * maxDistance / dotDistance;
        }

        int dotScreenX = centerX + dotX;
        int dotScreenY = centerY + dotY;

        spr.fillSmoothCircle(
            dotScreenX,
            dotScreenY,
            dotRadius,
            TFT_YELLOW);

        char gXStr[16];
        char gYStr[16];

        snprintf(gXStr, sizeof(gXStr), "%+.2f", gX);
        snprintf(gYStr, sizeof(gYStr), "%+.2f", gY);

        spr.drawString(gXStr, 80, 219);
        spr.drawString(gYStr, 240, 219);
    }
    else
    {
        spr.drawString("--", 80, 219);
        spr.drawString("--", 240, 219);
    }

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsUTCOffsetSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, STGUTCSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsUTCOffset(uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, STGUTC);

    spr.setTextDatum(MC_DATUM);
    spr.setFreeFont(&Orbitron_Light_32);
    spr.setTextSize(1);
    spr.setTextPadding(0);
    spr.setTextColor(TFT_YELLOW, TFT_BLACK);

    char offsetStr[16];

    snprintf(
        offsetStr,
        sizeof(offsetStr),
        "%+d",
        resources.utcOffset);

    spr.drawString(offsetStr, 155, 75);

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsGateSetupSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, STGATESel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsGateSetup(AppState state, uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, STGATE);

    spr.setTextDatum(MC_DATUM);
    spr.setFreeFont(&Orbitron_Light_32);
    spr.setTextSize(1);
    spr.setTextPadding(0);

    if (state == State_StartLineWidth)
    {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    else
    {
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
    }

    spr.drawNumber(resources.startLineWidth, 155, 72);

    if (state == State_GateWidth)
    {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    else
    {
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
    }

    spr.drawNumber(resources.gateWidth, 155, 137);

    if (state == State_DeltaMax)
    {
        spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    else
    {
        spr.setTextColor(TFT_WHITE, TFT_BLACK);
    }

    spr.drawFloat(resources.deltaMax, 1, 155, 203);

    switch (state)
    {
    case State_StartLineWidthSel:
        spr.fillCircle(90, 76, 5, TFT_WHITE);
        break;

    case State_GateWidthSel:
        spr.fillCircle(90, 144, 5, TFT_WHITE);
        break;

    case State_DeltaMaxSel:
        spr.fillCircle(90, 203, 5, TFT_WHITE);
        break;

    case State_StartLineWidth:
        spr.fillCircle(90, 76, 5, TFT_YELLOW);
        break;

    case State_GateWidth:
        spr.fillCircle(90, 144, 5, TFT_YELLOW);
        break;

    case State_DeltaMax:
        spr.fillCircle(90, 203, 5, TFT_YELLOW);
        break;

    default:
        break;
    }

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);

    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsAboutSel(uint8_t siv)
{
    spr.pushImage(0, 0, 320, 240, STGABTSel);
    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}

void TFTManager::drawSettingsAbout(uint8_t siv)
{
    if (!sprReady)
    {
        return;
    }

    spr.setSwapBytes(false);
    spr.pushImage(0, 0, 320, 240, STGABT);

    spr.setTextDatum(MC_DATUM);
    spr.setFreeFont(&Roboto_Thin_24);
    spr.setTextSize(1);
    spr.setTextPadding(0);
    spr.setTextColor(TFT_YELLOW, TFT_BLACK);

    spr.drawString("v.1.0.1", 155, 102);
    spr.drawString("Bartosz Czech", 155, 160);
    spr.drawString("23.09.2026", 155, 210);

    drawBattery();
    drawSDStatus();
    drawSivStatus(siv);
    spr.pushSprite(0, 0);
}
