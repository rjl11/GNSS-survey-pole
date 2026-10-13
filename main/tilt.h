#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define ROD_LENGTH_M 2.0   // antenna phase center -> rod tip

typedef struct {
    double lat, lon;     // degrees
    double alt;          // same datum as input altitude, meters
    double dN, dE, dD;   // antenna -> tip offset in NED, meters
} tip_position_t;

// Angles in degrees. Sensor frame (TransducerM): x forward, y left, z up
// (right-handed). Rotations are right-handed about each axis, applied ZYX
// (yaw, pitch, roll); yaw is counter-clockwise from MAGNETIC north.
// Rod runs along sensor z, antenna on top, tip at the bottom.
// declination_deg is east-positive (true = magnetic + declination).
tip_position_t tilt_compensate(double lat_deg, double lon_deg, double alt_m,
                               double roll_deg, double pitch_deg, double yaw_deg,
                               double declination_deg, double rod_len_m);

#ifdef __cplusplus
}
#endif
