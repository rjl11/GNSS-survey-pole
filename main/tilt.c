#include "tilt.h"
#include <math.h>

#define WGS84_A   6378137.0
#define WGS84_E2  0.00669437999014
#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)

tip_position_t tilt_compensate(double lat_deg, double lon_deg, double alt_m,
                               double roll_deg, double pitch_deg, double yaw_deg,
                               double declination_deg, double rod_len_m)
{
    double phi   = roll_deg  * DEG2RAD;
    double theta = pitch_deg * DEG2RAD;
    // CCW yaw from magnetic north -> CCW yaw from true north
    double psi   = (yaw_deg - declination_deg) * DEG2RAD;

    double sphi = sin(phi),   cphi = cos(phi);
    double sth  = sin(theta), cth  = cos(theta);
    double spsi = sin(psi),   cpsi = cos(psi);

    // v = Rz(psi) * Ry(theta) * Rx(phi) * [0, 0, -L] in a north-west-up frame
    double dN_ = -rod_len_m * (cphi * sth * cpsi + sphi * spsi);
    double dW_ =  rod_len_m * (sphi * cpsi - cphi * sth * spsi);
    double dU_ = -rod_len_m * (cphi * cth);

    tip_position_t o;
    o.dN = dN_;
    o.dE = -dW_;     // west -> east
    o.dD = -dU_;     // up -> down (positive when the tip is below the antenna)

    double lat_rad = lat_deg * DEG2RAD;
    double s  = sin(lat_rad);
    double w  = sqrt(1.0 - WGS84_E2 * s * s);
    double Rn = WGS84_A / w;                              // prime vertical radius
    double Rm = WGS84_A * (1.0 - WGS84_E2) / (w * w * w); // meridian radius

    o.lat = lat_deg + (o.dN / Rm) * RAD2DEG;
    o.lon = lon_deg + (o.dE / (Rn * cos(lat_rad))) * RAD2DEG;
    o.alt = alt_m - o.dD;
    return o;
}
