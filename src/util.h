#ifndef MY_UTIL_H
#define MY_UTIL_H

#include <vector>
#include <cmath>
#include <math.h>

#define _USE_MATH_DEFINES

using std::vector;

// Constants
const double TIME_INTERVAL = 0.02; // (seconds)
// multiplication coefficient for conversion from m/s to mph
const double MS_TO_MPH     = 2.24;
/** Useful
  22 m/s = 49.2 mph
  20 m/s = 44.7
  15 m/s = 33.5
  10 m/s = 22.4
  1  m/s = 2.24 mph
*/

// GAPS
const double FRONT_BUFFER              = 35.0; // meters; 
const double Min_dist_to_front_car           = FRONT_BUFFER - 10;
const double LANE_CHANGE_BUFFER_FRONT = 20.0; // meters
const double LANE_CHANGE_BUFFER_BACK  = 13.0; // A shorter back gap is OK for lane change

const double SPACING      = 30;    // Spacing (m) in waypoints

// LANE ID
const double LEFT_LANE    = 0;
const double CENTER_LANE  = 1;
const double RIGHT_LANE   = 2;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// util functions from main.cpp
double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);


#endif // MY_UTIL_H