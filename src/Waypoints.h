#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <vector>

#include "util.h"
#include "spline.h"

using std::vector;

class Waypoints {
  // for saving map waypoints

public:
  Waypoints(vector<double> x, vector<double> y,
            vector<double> s,
            vector<double> d_x, vector<double> d_y);

  ~Waypoints() {};

  vector<double> map_x_;
  vector<double> map_y_;
  vector<double> map_s_;
  vector<double> map_d_x_;
  vector<double> map_d_y_;

  // obtain next waypoint, provided values of x, y, angle theta
  // it uses map coords inside the method
  int GetNextWaypoint(double x, double y, double theta);

};


#endif // WAYPOINTS_H