#include "Waypoints.h"

Waypoints::Waypoints(vector<double> x, vector<double> y,
                  vector<double> s, vector<double>d_x, vector<double>d_y):
                  map_x_(x), map_y_(y), map_s_(s), map_d_x_(d_x), map_d_y_(d_y)
{
  // intentionally blank, no operations
  ;
}

// obtain next waypoint, provided values of x, y, and theta (angle)
// it uses map coords inside method
int Waypoints::GetNextWaypoint(double x, double y, double theta)
{
  // calls helper function from util.h
  return NextWaypoint(x, y, theta, this->map_x_, this->map_y_);
}