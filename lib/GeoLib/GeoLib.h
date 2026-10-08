#pragma once
#include <math.h>

class GeoLib {
private:
    double lat0, lon0;
    const double a = 6378137.0;
    const double e2 = 0.00669437999014;

    void geodeticToECEF(double lat_rad, double lon_rad, double &x, double &y, double &z) {
        double sin_lat = sin(lat_rad), cos_lat = cos(lat_rad);
        double sin_lon = sin(lon_rad), cos_lon = cos(lon_rad);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        
        x = N * cos_lat * cos_lon;
        y = N * cos_lat * sin_lon;
        z = N * (1.0 - e2) * sin_lat;
    }

public:
    GeoLib() : lat0(0.0), lon0(0.0) {}

    void LocalCartesian(double lat_rad, double lon_rad) {
        lat0 = lat_rad;
        lon0 = lon_rad;
    }

    void Forward(double lat_rad, double lon_rad, double &east, double &north) {
        double x0, y0, z0, x, y, z;
        
        geodeticToECEF(lat0, lon0, x0, y0, z0);
        geodeticToECEF(lat_rad, lon_rad, x, y, z);

        double dx = x - x0;
        double dy = y - y0;
        double dz = z - z0;

        double sin_lat0 = sin(lat0), cos_lat0 = cos(lat0);
        double sin_lon0 = sin(lon0), cos_lon0 = cos(lon0);

        east  = -sin_lon0 * dx + cos_lon0 * dy;
        north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz;
    }
};