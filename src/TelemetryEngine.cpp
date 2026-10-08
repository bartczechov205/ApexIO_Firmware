#include "TelemetryEngine.h"
#include "AppResources.h"
#include "MathUtils.h"
#include <Arduino.h>
#include <algorithm>
#include "GeoLib.h"
#include <vector>

extern AppResources resources;

void TelemetryEngine::updateTelemetryEngine(SystemData &systemData)
{
    localSystemData = systemData;
}

double median9(const double values[9])
{
    double sortedValues[9];
    std::copy(values, values + 9, sortedValues);
    std::sort(sortedValues, sortedValues + 9);
    return sortedValues[4];
}

static double updateSpeedMedian(double (&samples)[9], double speedKmh)
{
    for (int i = 0; i < 8; ++i)
    {
        samples[i] = samples[i + 1];
    }

    samples[8] = floor(speedKmh * 10.0) / 10.0;

    return median9(samples);
}

void resetSpeedTable(double (&table)[9])
{
    std::fill(std::begin(table), std::end(table), 0.0);
}

void TelemetryEngine::updateLapDelta(double Qx, double Qy, uint32_t currentTimeSample)
{
    if (gates.empty() || bestRoute.empty() || currentGateNumber >= bestRoute.size())
    {
        return;
    }

    double Ax = 0.0;
    double Ay = 0.0;
    uint32_t prevGateTimeBest = 0;

    if (currentGateNumber > 0)
    {
        GateLapTimerPoint previousGate = gates[currentGateNumber - 1];

        Ax = (previousGate.P1x + previousGate.P2x) / 2.0;
        Ay = (previousGate.P1y + previousGate.P2y) / 2.0;

        prevGateTimeBest = bestRoute[currentGateNumber - 1].lapTime;
    }
    else
    {
        Ax = (strLineP1.x + strLineP2.x) / 2.0;
        Ay = (strLineP1.y + strLineP2.y) / 2.0;
        prevGateTimeBest = 0;
    }

    double Bx = (gates[currentGateNumber].P1x + gates[currentGateNumber].P2x) / 2.0;
    double By = (gates[currentGateNumber].P1y + gates[currentGateNumber].P2y) / 2.0;

    double vectorX = Bx - Ax;
    double vectorY = By - Ay;

    double vectorL = sqrt(vectorX * vectorX + vectorY * vectorY);

    if (vectorL < 0.0001)
    {
        return;
    }

    double vx = vectorX / vectorL;
    double vy = vectorY / vectorL;

    double progress = ((Qx - Ax) * vx + (Qy - Ay) * vy) / vectorL;

    if (progress < 0.0)
    {
        progress = 0.0;
    }
    else if (progress > 1.0)
    {
        progress = 1.0;
    }

    uint32_t deltaTimeBtwGates = bestRoute[currentGateNumber].lapTime - prevGateTimeBest;
    double referenceTime = prevGateTimeBest + progress * deltaTimeBtwGates;
    double currentGateTime = currentTimeSample - startTime;

    lapTimerDataBox.delta = (currentGateTime - referenceTime) / 1000000.0;
}

void TelemetryEngine::dragModeRestart()
{
    distance_km = 0.0;
    lastdistance_km = 0.0;
    lastTime = 0;
    stopTime = 0;
    startMeasurementTime = 0;
    breakingStartDisM = 0.0;
    brakingDistanceM = 0.0;
    dragState = Drag_Waiting;
    isStoped = false;
    resetSpeedTable(speedTabelDM);
    isReady0100 = false;
    isReady60ft = false;
    isReady100 = false;
    isReady200 = false;
    isReady400 = false;
    isReadyBrkDis = false;
    hasStartPoint = false;
    dragModeDataBox = DragModeData{};
    dragModeDataBox.isMeasuring = false;
    dragModeDataBox.r60ft = false;
    dragModeDataBox.r100m = false;
    dragModeDataBox.r200m = false;
    dragModeDataBox.r400m = false;
    dragModeDataBox.r0100 = false;
    dragModeDataBox.status = DragStatus::DragStop;
    dragModeDataBox.Vmax = 0.0;
}

void TelemetryEngine::processDataForState(AppStateManager &appStateManager)
{
    const AppState currentState = appStateManager.getState();
    const bool okPressed = appStateManager.isDragOkPressed();
    okPressedReceived = okPressed;

    if (previousState == State_LapTimer && currentState != State_LapTimer)
    {
        lapTimerRestart();

        xQueueSend(resources.lapTimerSDQueue, &lapTimerSDDataBox, portMAX_DELAY);
    }

    switch (currentState)
    {
    case State_LiveTele:
    {
        if (previousState != State_LiveTele)
        {
            distance_km = 0;
            lastdistance_km = 0;
            lastTime = 0;
            resetSpeedTable(speedTabelLT);
        }
        updateLiveTele();
        break;
    }
    case State_DragMode:
    {
        if (previousState != State_DragMode)
        {
            dragModeRestart();
        }
        updateDragMode();
        break;
    }
    case State_LapTimer:
    {
        if (previousState != State_LapTimer)
        {
            resetSpeedTable(speedTabelLapT);
        }
        updateLapTimer();
        break;
    }
    case State_Settings_Axis_Calib:
        updateAxisCalib();
        break;
    default:
        break;
    }

    previousState = currentState;
}

void TelemetryEngine::lapTimerRestart()
{
    lapTimerState = LapTimer_Waiting;
    hasPreviousPosition = false;
    lineArmed = false;
    lapTimerDataBox = LapTimerData{};

    gates.clear();
    currentRoute.clear();
    bestRoute.clear();

    lapTimerSDDataBox.isMeasuring = false;
    isFirstSampleLogged = false;
    lastLogTimeMs = 0;
    hasGatePosition = false;

    currentGateNumber = 0;
    totalMoveL = 0.0;
    startTime = 0;
}

void TelemetryEngine::lapTimerCrossingGates(
    double t,
    double check_t,
    uint32_t currentTimeSample,
    double dx,
    double dy)
{
    double last_t_delta = t;

    while (lapTimerDataBox.isLapValid && currentGateNumber >= 0 && static_cast<size_t>(currentGateNumber) < gates.size())
    {
        int crossedGateIndex = -1;
        double firstCrossingT = 0.0;

        for (size_t i = static_cast<size_t>(currentGateNumber); i < gates.size(); ++i)
        {
            const double ax = gates[i].P2x - gates[i].P1x;
            const double ay = gates[i].P2y - gates[i].P1y;

            const double mx = gates[i].P1x - previousPosition.x;
            const double my = gates[i].P1y - previousPosition.y;

            const double D_delta = dx * ay - dy * ax;

            if (D_delta >= -0.0001)
            {
                continue;
            }

            const double t_delta = (mx * ay - my * ax) / D_delta;
            const double s_delta = (mx * dy - my * dx) / D_delta;

            if (!(t_delta > last_t_delta &&
                  t_delta <= 1.0 &&
                  t_delta <= check_t &&
                  s_delta >= 0.0 &&
                  s_delta <= 1.0))
            {
                continue;
            }

            if (crossedGateIndex < 0 || t_delta < firstCrossingT)
            {
                crossedGateIndex = static_cast<int>(i);
                firstCrossingT = t_delta;
            }
        }

        if (crossedGateIndex < 0)
        {
            break;
        }

        if (crossedGateIndex != currentGateNumber)
        {
            lapTimerDataBox.isLapValid = false;
            break;
        }

        crossingTime = previousTimeSample + static_cast<uint32_t>(firstCrossingT * (currentTimeSample - previousTimeSample));

        DeltaLapTimerPoint point{};
        point.nr = currentGateNumber;
        point.lapTime = static_cast<uint32_t>(crossingTime - startTime);

        currentRoute.push_back(point);

        last_t_delta = firstCrossingT;
        currentGateNumber++;
    }
}

uint32_t utcmillisecondsconverter(gpsExtraData &gps)
{
    constexpr int64_t NS_PER_SECOND = 1000000000LL;
    constexpr int64_t NS_PER_DAY = 86400LL * NS_PER_SECOND;
    constexpr int64_t MS_PER_DAY = 86400000LL;

    const int64_t seconds =
        static_cast<int64_t>((gps.hr)+(resources.utcOffset)) * 3600LL +
        static_cast<int64_t>(gps.min) * 60LL +
        gps.sec;

    int64_t totalNs = seconds * NS_PER_SECOND + gps.ns;

    totalNs %= NS_PER_DAY;
    if (totalNs < 0)
        totalNs += NS_PER_DAY;

    const int64_t roundedMs = (totalNs + 500000LL) / 1000000LL;

    return static_cast<uint32_t>(roundedMs % MS_PER_DAY);
}


void TelemetryEngine::updateAxisCalib()
{
    axisCalibDataBox.gX = -localSystemData.rawImuData.acclY / 9.81f;
    axisCalibDataBox.gY = -localSystemData.rawImuData.acclX / 9.81f;

    xQueueOverwrite(resources.axisCalibQueue, &axisCalibDataBox);
}

void TelemetryEngine::updateLiveTele()
{
    double vN = localSystemData.ekfOutputData.vN;
    double vE = localSystemData.ekfOutputData.vE;
    double gX = localSystemData.rawImuData.acclX;
    double gY = localSystemData.rawImuData.acclY;
    double alt_m = localSystemData.ekfOutputData.alt_m;
    double hdg = localSystemData.rawGpsExtraData.hdg;

    liveTeleDataBox.speedKmh = updateSpeedMedian(speedTabelLT, sqrt(vN * vN + vE * vE) * 3.6);

    if (liveTeleDataBox.speedKmh <= 1.0 && fabs(gY / 9.81) < 0.10)
    {
        liveTeleDataBox.speedKmh = 0;
    }

    liveTeleDataBox.gX = gX / 9.81;
    liveTeleDataBox.gY = gY / 9.81;

    liveTeleDataBox.alt_m = alt_m - 40;

    nowTime = micros();
    if (lastTime == 0)
    {
        lastTime = nowTime;
    }
    dt_s = (nowTime - lastTime) / 1000000.0;
    lastTime = nowTime;
    distance_km += liveTeleDataBox.speedKmh * (dt_s / 3600.0);
    liveTeleDataBox.dis = distance_km;

    hdg = (hdg / 100000.0);

    if (liveTeleDataBox.speedKmh < 1.0)
    {
        liveTeleDataBox.hdg = "P";
    }
    else if (hdg >= 337.5 || hdg < 22.5)
    {
        liveTeleDataBox.hdg = "N";
    }
    else if (hdg >= 22.5 && hdg < 67.5)
    {
        liveTeleDataBox.hdg = "NE";
    }
    else if (hdg >= 67.5 && hdg < 112.5)
    {
        liveTeleDataBox.hdg = "E";
    }
    else if (hdg >= 112.5 && hdg < 157.5)
    {
        liveTeleDataBox.hdg = "SE";
    }
    else if (hdg >= 157.5 && hdg < 202.5)
    {
        liveTeleDataBox.hdg = "S";
    }
    else if (hdg >= 202.5 && hdg < 247.5)
    {
        liveTeleDataBox.hdg = "SW";
    }
    else if (hdg >= 247.5 && hdg < 292.5)
    {
        liveTeleDataBox.hdg = "W";
    }
    else if (hdg >= 292.5 && hdg < 337.5)
    {
        liveTeleDataBox.hdg = "NW";
    }

    xQueueOverwrite(resources.liveTelemetryQueue, &liveTeleDataBox);
}

void TelemetryEngine::updateDragMode()
{
    double vN = localSystemData.ekfOutputData.vN;
    double vE = localSystemData.ekfOutputData.vE;
    double gX = localSystemData.rawImuData.acclX;

    Vm = sqrt((vN * vN) + (vE * vE));
    Vkph = Vm * 3.6;

    dragModeDataBox.speedKmh = updateSpeedMedian(speedTabelDM, Vkph);

    if (dragModeDataBox.speedKmh <= 1.0 && fabs(gX / 9.81) < 0.05)
    {
        dragModeDataBox.speedKmh = 0;
    }

    dragModeDataBox.gX = gX / 9.81;

    Serial.println(dragModeDataBox.speedKmh);
    Serial.println(dragModeDataBox.gX);

    switch (dragState)
    {
    case Drag_Waiting:
        dragModeDataBox.status = DragStatus::DragStop;

        if (dragModeDataBox.speedKmh <= 0.2)
        {
            if (!isStoped)
            {
                stopTime = micros();
                isStoped = true;
            }

            if ((micros() - stopTime) >= 10000000)
            {
                dragState = Drag_Ready;
            }
        }
        else
        {
            isStoped = false;
            stopTime = 0;
        }
        break;

    case Drag_Ready:
        dragModeDataBox.status = DragStatus::DragReady;

        if (dragModeDataBox.speedKmh >= 0.2)
        {
            startMeasurementTime = micros();
            dragModeDataBox.timer = 0.0;
            dragState = Drag_Measuring;
        }
        break;

    case Drag_Measuring:

        dragModeDataBox.status = DragStatus::DragMeasuring;
        dragModeDataBox.isMeasuring = true;

        nowTime = micros();
        if (lastTime == 0)
        {
            lastTime = nowTime;
        }
        dt_s = (nowTime - lastTime) / 1000000.0;
        lastTime = nowTime;
        distance_km += Vkph * (dt_s / 3600.0);
        dragModeDataBox.dis = distance_km;
        dragModeDataBox.timer = (micros() - startMeasurementTime) / 1000000.0;

        if (Vkph >= dragModeDataBox.Vmax)
        {
            dragModeDataBox.Vmax = Vkph;
        }

        if (!isReady0100)
        {
            if (Vkph >= 100)
            {
                dragModeDataBox.t0100 = (micros() - startMeasurementTime) / 1000000.0;
                dragModeDataBox.r0100 = true;
                isReady0100 = true;
            }
        }

        if (distance_km >= 0.018288)
        {
            if (!isReady60ft)
            {
                dragModeDataBox.t60ft = (micros() - startMeasurementTime) / 1000000.0;
                dragModeDataBox.r60ft = true;
            }
            isReady60ft = true;
        }

        if (distance_km >= 0.1)
        {
            if (!isReady100)
            {
                dragModeDataBox.t100m = (micros() - startMeasurementTime) / 1000000.0;
                dragModeDataBox.r100m = true;
            }
            isReady100 = true;
        }

        if (distance_km >= 0.2)
        {
            if (!isReady200)
            {
                dragModeDataBox.t200m = (micros() - startMeasurementTime) / 1000000.0;
                dragModeDataBox.r200m = true;
            }
            isReady200 = true;
        }

        if (distance_km >= 0.4)
        {
            if (!isReady400)
            {
                dragModeDataBox.t400m = (micros() - startMeasurementTime) / 1000000.0;
                dragModeDataBox.r400m = true;
            }
            isReady400 = true;
            dragState = Drag_Finished;
        }
        break;

    case Drag_Finished:
        dragModeDataBox.status = DragStatus::DragFinished;
        dragModeDataBox.isMeasuring = false;

        nowTime = micros();
        if (lastTime == 0)
        {
            lastTime = nowTime;
        }
        dt_s = (nowTime - lastTime) / 1000000.0;
        lastTime = nowTime;
        brakingDistanceM += Vkph * (dt_s / 3.6);

        if (!isReadyBrkDis)
        {
            if (!hasStartPoint && Vkph > 10.0 && dragModeDataBox.gX >= 0.4)
            {
                breakingStartDisM = brakingDistanceM;
                hasStartPoint = true;
            }
            else if (hasStartPoint && Vkph < 1.0)
            {
                dragModeDataBox.brakingDis = brakingDistanceM - breakingStartDisM;
                isReadyBrkDis = true;
            }
        }

        if (okPressedReceived)
        {
            dragModeRestart();
        }
        break;
    }

    xQueueOverwrite(resources.dragModeQueue, &dragModeDataBox);
}

void TelemetryEngine::updateLapTimer()
{
    uint32_t currentTimeSample = micros();

    double lat = localSystemData.ekfOutputData.lat_rad;
    double lon = localSystemData.ekfOutputData.lon_rad;
    double vN = localSystemData.ekfOutputData.vN;
    double vE = localSystemData.ekfOutputData.vE;
    double gX = localSystemData.rawImuData.acclX;
    double gY = localSystemData.rawImuData.acclY;

    Vm = sqrt((vN * vN) + (vE * vE));
    Vkph = Vm * 3.6;

    lapTimerDataBox.speedKmh = updateSpeedMedian(speedTabelLapT, Vkph);

    if (lapTimerDataBox.speedKmh <= 1.0 && fabs(gY / 9.81) < 0.05)
    {
        lapTimerDataBox.speedKmh = 0;
    }

    lapTimerDataBox.gX = gX / 9.81;
    lapTimerDataBox.gY = gY / 9.81;

    switch (lapTimerState)
    {
    case LapTimer_Waiting:

        lapTimerDataBox.status = LapStatus::Waiting;

        if (okPressedReceived)
        {
            geoLib.LocalCartesian(lat, lon);
            lapTimerState = LapTimer_Calibration;
        }
        break;

    case LapTimer_Calibration:
        lapTimerDataBox.status = LapStatus::Calibration;

        if (lapTimerDataBox.speedKmh >= 1.0)
        {
            geoLib.Forward(lat, lon, calibP1.x, calibP1.y);

            calibL = sqrt((calibP1.x * calibP1.x) + (calibP1.y * calibP1.y));

            if (calibL > 10)
            {
                uX = calibP1.x / calibL;
                uY = calibP1.y / calibL;

                strLineP1.x = -uY * resources.startLineWidth;
                strLineP1.y = uX * resources.startLineWidth;
                strLineP2.x = uY * resources.startLineWidth;
                strLineP2.y = -uX * resources.startLineWidth;

                lapTimerState = LapTimer_LapZero;
            }
        }
        break;

    case LapTimer_LapZero:
    {
        lapTimerDataBox.status = LapStatus::LapZero;

        double Qx = 0.0;
        double Qy = 0.0;

        geoLib.Forward(lat, lon, Qx, Qy);

        if (hasPreviousPosition == false)
        {
            previousPosition.x = Qx;
            previousPosition.y = Qy;
            previousTimeSample = currentTimeSample;
            hasPreviousPosition = true;

            hasGatePosition = false;
            totalMoveL = 0.0;
        }

        double dx = Qx - previousPosition.x;
        double dy = Qy - previousPosition.y;

        double ex = strLineP2.x - strLineP1.x;
        double ey = strLineP2.y - strLineP1.y;

        double wx = strLineP1.x - previousPosition.x;
        double wy = strLineP1.y - previousPosition.y;

        double D = dx * ey - dy * ex;
        double t = 0.0;
        double s = 0.0;

        if (fabs(D) > 0.0001)
        {
            t = (wx * ey - wy * ex) / D;
            s = (wx * dy - wy * dx) / D;

            bool forward = (dx * uX + dy * uY) > 0.0;

            if (forward && t > 0.0 && t <= 1.0 && s >= 0.0 && s <= 1.0)
            {
                startTime = previousTimeSample + static_cast<uint32_t>(t * (currentTimeSample - previousTimeSample));

                lineArmed = false;

                currentRoute.clear();
                currentGateNumber = 0;
                lapTimerDataBox.isLapValid = !gates.empty();

                lapTimerState = LapTimer_Measuring;

                lapTimerCrossingGates(t, 1.0, currentTimeSample, dx, dy);

                previousPosition.x = Qx;
                previousPosition.y = Qy;
                previousTimeSample = currentTimeSample;

                break;
            }
        }

        if (lapTimerDataBox.speedKmh < 1.0)
        {
            hasGatePosition = false;
            totalMoveL = 0.0;
        }
        else if (!hasGatePosition)
        {
            gatePosition.x = Qx;
            gatePosition.y = Qy;

            hasGatePosition = true;
            totalMoveL = 0.0;
        }
        else
        {
            double gateDx = Qx - gatePosition.x;
            double gateDy = Qy - gatePosition.y;

            totalMoveL = sqrt(
                gateDx * gateDx + gateDy * gateDy);

            if (totalMoveL >= 5.0)
            {
                double ux = gateDx / totalMoveL;
                double uy = gateDy / totalMoveL;
                double nx = -uy;
                double ny = ux;

                GateLapTimerPoint newGate{};
                newGate.nr = gates.size();
                newGate.P1x = Qx + nx * resources.gateWidth;
                newGate.P1y = Qy + ny * resources.gateWidth;
                ;
                newGate.P2x = Qx - nx * resources.gateWidth;
                ;
                newGate.P2y = Qy - ny * resources.gateWidth;
                ;

                gates.push_back(newGate);

                gatePosition.x = Qx;
                gatePosition.y = Qy;
                totalMoveL = 0.0;
            }
        }

        previousTimeSample = currentTimeSample;
        previousPosition.x = Qx;
        previousPosition.y = Qy;

        break;
    }

    case LapTimer_Measuring:
    {

        lapTimerSDDataBox.siv = localSystemData.rawGpsExtraData.siv;
        lapTimerSDDataBox.timeMs = utcmillisecondsconverter(localSystemData.rawGpsExtraData);
        lapTimerSDDataBox.lat = localSystemData.ekfOutputData.lat_rad * (180.0 / M_PI) * 60.0;
        lapTimerSDDataBox.lon = -localSystemData.ekfOutputData.lon_rad * (180.0 / M_PI) * 60.0;
        lapTimerSDDataBox.velocityKmh = Vkph;
        lapTimerSDDataBox.headingDeg = localSystemData.rawGpsExtraData.hdg / 100000.0;
        lapTimerSDDataBox.heightM = localSystemData.rawGpsExtraData.alt_mm / 1000.0;
        lapTimerSDDataBox.verticalVelocity = (-localSystemData.ekfOutputData.vD);
        lapTimerSDDataBox.samplePeriodS = 0.1;
        lapTimerSDDataBox.hr = localSystemData.rawGpsExtraData.hr;
        lapTimerSDDataBox.min = localSystemData.rawGpsExtraData.min;

        if (okPressedReceived)
        {
            lapTimerRestart();
            xQueueSend(resources.lapTimerSDQueue, &lapTimerSDDataBox, portMAX_DELAY);
            break;
        }

        lapTimerSDDataBox.isMeasuring = true;
        lapTimerDataBox.isMeasuring = true;

        lapTimerDataBox.counter = (micros() - startTime) / 1000000.0;

        constexpr double rearmDistanceM = 3.0;
        double Qx = 0.0;
        double Qy = 0.0;

        geoLib.Forward(lat, lon, Qx, Qy);

        updateLapDelta(Qx, Qy, currentTimeSample);

        double distanceBeforeLine = Qx * uX + Qy * uY;

        if (distanceBeforeLine <= -rearmDistanceM)
        {
            lineArmed = true;
        }

        double dx = Qx - previousPosition.x;
        double dy = Qy - previousPosition.y;
        double ex = strLineP2.x - strLineP1.x;
        double ey = strLineP2.y - strLineP1.y;
        double wx = strLineP1.x - previousPosition.x;
        double wy = strLineP1.y - previousPosition.y;
        double D = dx * ey - dy * ex;

        double t = 0.0;
        double s = 0.0;
        double check_t = 1.0;
        bool crossedFinish = false;

        if (fabs(D) > 0.0001)
        {
            t = (wx * ey - wy * ex) / D;
            s = (wx * dy - wy * dx) / D;

            bool forward = (dx * uX + dy * uY) > 0.0;

            if (lineArmed && forward && t > 0.0 && t <= 1.0)
            {
                lineArmed = false;

                if (s >= 0.0 && s <= 1.0)
                {
                    crossedFinish = true;
                }
            }
        }

        if (crossedFinish)
        {
            check_t = t;
        }

        lapTimerCrossingGates(0.0, check_t, currentTimeSample, dx, dy);

        if (crossedFinish)
        {
            crossingTime = previousTimeSample + static_cast<uint32_t>(t * (currentTimeSample - previousTimeSample));

            lapTimerDataBox.lastTimeS = (crossingTime - startTime) / 1000000.0;

            lapTimerDataBox.isLapValid = lapTimerDataBox.isLapValid && !gates.empty() && currentRoute.size() == gates.size();

            if (lapTimerDataBox.isLapValid && (lapTimerDataBox.bestTimeS == 0.0 || lapTimerDataBox.lastTimeS < lapTimerDataBox.bestTimeS))
            {
                lapTimerDataBox.bestTimeS = lapTimerDataBox.lastTimeS;
                bestRoute = currentRoute;
            }

            startTime = crossingTime;
            currentRoute.clear();
            currentGateNumber = 0;
            totalMoveL = 0.0;

            lapTimerDataBox.isLapValid = !gates.empty();

            lapTimerCrossingGates(t, 1.0, currentTimeSample, dx, dy);
        }

        previousPosition.x = Qx;
        previousPosition.y = Qy;
        previousTimeSample = currentTimeSample;

        break;
    }

    default:
        break;
    }

    xQueueOverwrite(resources.lapTimerQueue, &lapTimerDataBox);

    if (lapTimerSDDataBox.isMeasuring)
    {
        if ((!isFirstSampleLogged) || (lapTimerSDDataBox.timeMs != lastLogTimeMs))
        {
            if (!isFirstSampleLogged)
            {
                lapTimerSDDataBox.samplePeriodS = 0.100f;
            }
            else
            {
                lapTimerSDDataBox.samplePeriodS =
                    (lapTimerSDDataBox.timeMs - lastLogTimeMs) / 1000.0f;
            }

            if (xQueueSend(resources.lapTimerSDQueue, &lapTimerSDDataBox, 0) == pdTRUE)
            {
                lastLogTimeMs = lapTimerSDDataBox.timeMs;
                isFirstSampleLogged = true;
            }
        }
    }
    else
    {
        isFirstSampleLogged = false;
        lastLogTimeMs = 0;
    }
}