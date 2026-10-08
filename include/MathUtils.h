#pragma once

#include <Arduino.h>

class MathUtils
{

public:
    float convertToMPerS(float x)
    {
        return x / 1000;
    }

    double convertGeoToRadians(double x)
    {
        return (x / 10000000) * (PI / 180);
    }
    double MiliMeterstoMeters(double x)
    {
        return x / 1000;
    }
};