#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>
#include <iomanip>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "util.h"
#include "Waypoints.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Return Lane number (0 - Left, 1 - Center, 2 - Right)
double getCarLane(double car_d) {
  double lane = 1; // default CENTER lane
  if (car_d > 0 && car_d < 4) {
    lane = 0;    // LEFT
  } else if (car_d >= 4 && car_d <= 8) {
    lane = 1;   // CENTER
  } else if (car_d > 8 && car_d <= 12) {
    lane = 2;    // RIGHT
  }
  return lane;
}

// return textual name of lane, from its code
string getLaneInfo(double lane_num) {
  if (lane_num == 0) { return "LEFT"; }
  else if (lane_num == 1) { return "CENTER"; }
  else if (lane_num == 2) { return "RIGHT"; }
  else { return "NONE"; }
}

inline string yes_no(bool value) {
  if (value) { return "YES"; }
  else { return "NO"; }
}


// Path Planner --  Return TRUE if safe to change into given lane
bool is_lane_safe(const int num_points,     // num of points to project speed for
                  const double ego_car_s,  // Ego Car's s
                  const double ref_vel,      // Ego Car's reference velocity
                  const double check_lane, // Lane to look for
                  const vector<vector<double> > &sensor_fusion_data) {
  bool ok_to_change = false;      // should we move into the check_lane?

  double SHORTEST_FRONT = 100000; // Really big
  double SHORTEST_BACK = -100000;

  cout << "   Front buffer (m): " << LANE_CHANGE_BUFFER_FRONT
       << ",  Back buffer: " << LANE_CHANGE_BUFFER_BACK << endl;

  // Calculate the closest Front and Back gaps
  for (int i = 0; i < sensor_fusion_data.size(); i++) {
    float d = sensor_fusion_data[i][6];    // d for a Traffic Car
    double other_car_lane = getCarLane(d); // lane of the Traffic Car
    // if a Traffic Car is in the lane to check
    if (other_car_lane == check_lane) {
      // get it's speed
      double vx = sensor_fusion_data[i][3];
      double vy = sensor_fusion_data[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      // get it's s displacement
      double check_car_s = sensor_fusion_data[i][5];

      // see how far Other Car will go in TIME_INTERVAL seconds
      // i.e. project its future s
      check_car_s += ((double) num_points * TIME_INTERVAL * check_speed);

      // see the gap from our Ego Car
      double dist_s = check_car_s - ego_car_s;  // WAS: ego_car_s
      // remove -ve sign
      double dist_pos = sqrt(dist_s * dist_s);

      // store the shortest gap
      // SHORTEST_S = min(dist_pos, SHORTEST_S);

      if (dist_s > 0) {                  // FRONT gap
        SHORTEST_FRONT = min(dist_s, SHORTEST_FRONT);
      } else if (dist_s <= 0) {          // BACK gap
        SHORTEST_BACK = max(dist_s, SHORTEST_BACK);
      }

      cout << "   gap (m): "
           << setprecision(5)
           << dist_s
           << ", closest front: "
           << setprecision(5)
           << SHORTEST_FRONT
           << ", closest back: "
           << setprecision(5)
           << SHORTEST_BACK
           << endl;
    }
  }    // for-each-Traffic-car
  cout << "   gap (m): "
       << " >>> Closest Front: "
       << setprecision(5)
       << SHORTEST_FRONT
       << ", closest Back: "
       << setprecision(5)
       << SHORTEST_BACK
       << " <<< "
       << endl;
  // Only if enough space in that lane, move to that lane
  if ((SHORTEST_FRONT > LANE_CHANGE_BUFFER_FRONT) &&
      (-1 * SHORTEST_BACK > LANE_CHANGE_BUFFER_BACK)) {
    ok_to_change = true;
  }

  cout << " Assessing Lane  : " << getLaneInfo(check_lane) << ", Is it Ok to change? " << yes_no(ok_to_change)
       << endl;
  return ok_to_change;

} // end is-lane-safe


// ============= MAIN ==============
int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Save waypoints
  Waypoints waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s,
                      map_waypoints_dx, map_waypoints_dy);

  int our_lane = CENTER_LANE;  // We are initially in center lane
  int lane_change_waypoint = 0;

  h.onMessage([&waypoints,
                  &our_lane, &lane_change_waypoint]
                  (uWS::WebSocket<uWS::SERVER> ws,
                   char *data, size_t length,
                   uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          double // maximum speed in mph
              max_speed = 50.0;
          double // reference speed in mph (a little less than maximum)
              ref_speed = max_speed - 2;

          json msgJson;

          // these values will be sent to sim at next step
          vector<double> next_x_vals;
          vector<double> next_y_vals;
//  BEGIN custom path planning -----------------------------------------------------
          // 1. reference coordinates for X, Y, Yaw - first initialized with values from the previous path
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          int path_size = previous_path_x.size();
          int next_waypoint = -1;

          // if previous path is too short or absent
          if (path_size < 2) {
            // then obtain next way point, from reference and map coordinates
            next_waypoint = waypoints.GetNextWaypoint(ref_x, ref_y, ref_yaw);
          } else {
            // or else take end point of the previous path as reference start

            // update reference coords
            ref_x = previous_path_x[path_size - 1];
            ref_y = previous_path_y[path_size - 1];
            // save reference coords of the previous path
            double ref_x_prev = previous_path_x[path_size - 2];
            double ref_y_prev = previous_path_y[path_size - 2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            // at last, obtain next way point, from reference and map coordinates
            next_waypoint = waypoints.GetNextWaypoint(ref_x, ref_y, ref_yaw);

            // obtain previous values of s and speed
            double dist = distance(ref_x, ref_y, ref_x_prev, ref_y_prev);

            car_s = end_path_s;
            car_speed = (dist / TIME_INTERVAL) * MS_TO_MPH;
          }

          // Sensor Fusion ----------------------------------------

          // calculate reference speed from provided nearby traffic
          bool too_near = false;
          bool need_to_change_lane = false;
          double closest_dist_s = 1e5; // some big number

          for (int i = 0; i < sensor_fusion.size(); ++i) {
            double other_vx = sensor_fusion[i][3];
            double other_vy = sensor_fusion[i][4];
            double other_car_s = sensor_fusion[i][5];

            double other_d = sensor_fusion[i][6];    // d of other car
            double other_lane = getCarLane(other_d);    // lane of other car

            // if other car is in the same lane as ours
            if (other_lane == our_lane) {
              double other_speed = sqrt(other_vx * other_vx + other_vy * other_vy);
              // predict position of other car in the future few steps
              other_car_s += (path_size * TIME_INTERVAL * other_speed);
              // calculate gap between front car and ours
              double front_gap = other_car_s - car_s;
              // check whether another car is before us and distance to it
              if (front_gap > 0 && front_gap < FRONT_BUFFER && front_gap < closest_dist_s) {
                closest_dist_s = front_gap;

                if (front_gap > Min_dist_to_front_car) {
                  // follow the car in front of ours
                  ref_speed = other_speed * MS_TO_MPH;

                  // if true, then try to change lane
                  need_to_change_lane = true;
                  cout << "Front gap: " << front_gap
                       << "\tReference speed (mph): "
                       << setprecision(4)
                       << ref_speed
                       << ", current speed (mph): "
                       << car_speed
                       << endl;
                } else {
                  // distance to front car's became too small,
                  // so we need to slow down before front car
                  ref_speed = other_speed * MS_TO_MPH - 5.0;
                  too_near = true;
                  // We certainly need to change lane:
                  need_to_change_lane = true;
                  cout << "Distance to front car's became too small = " << front_gap
                       << ", \treference speed (mph) = "
                       << setprecision(4)
                       << ref_speed
                       << ", current speed (mph) = "
                       << car_speed
                       << endl;
                }
                cout << "   need to change lane = " << yes_no(need_to_change_lane) << endl;
              }
            } // in our lane?
          } // sensor-fusion

          // START Lane Change: Find Goal Lane (if needed)
          int delta_wp = next_waypoint - lane_change_waypoint;
          int remain_wp = delta_wp % waypoints.map_x_.size();

          if (need_to_change_lane && remain_wp > 2) {
            cout << "  assessing lane change from: "
                 << getLaneInfo(our_lane)
                 << ", at s: " << car_s << endl;
            bool changed_lane = false;
            // First, assess Left lane
            if (our_lane != LEFT_LANE && !changed_lane) {
              bool lane_is_safe = true;

              // Check whether we can change to the Left
              lane_is_safe = is_lane_safe(path_size,
                                          car_s,
                                          ref_speed,
                                          our_lane - 1, // To the Left of Current
                                          sensor_fusion);

              // it is safe to move Left
              if (lane_is_safe) {
                changed_lane = true;
                our_lane -= 1;  // move Left by one lane
                lane_change_waypoint = next_waypoint;
              }
            }
            // Second, attempt Right Lane
            if (our_lane != RIGHT_LANE && !changed_lane) {
              bool lane_is_safe = true;

              // Check whether we can change to the RIGHT
              lane_is_safe = is_lane_safe(path_size,
                                          car_s,
                                          ref_speed,
                                          our_lane + 1,  // to the Right of Current
                                          sensor_fusion);

              // it is safe to move Right
              if (lane_is_safe) {
                changed_lane = true;
                our_lane += 1;  // go Right by one lane
                lane_change_waypoint = next_waypoint;
              }
            }
            cout << " Current Lane: "
                 << getLaneInfo(our_lane)
                 << ",  changed_lane: "
                 << yes_no(changed_lane)
                 << ", s: " << car_s
                 << endl;
          } // if change lane
          // END lane change

          // since we have goal lane & reference speed, we can compute Smooth path:
          // make list of broadly spaced (x,y) anchor points, spaced regularly at 30meters;
          // estimate the waypoints by spline,
          // and populate by more points for controlling speed
          vector<double> anchor_x_pts;
          vector<double> anchor_y_pts;

          // When previous path is nearly void, take the car itself as reference starting point
          if (path_size < 2) {

            // use two points making the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            // append to list of anchor points
            anchor_x_pts.push_back(prev_car_x);
            anchor_x_pts.push_back(car_x);

            anchor_y_pts.push_back(prev_car_y);
            anchor_y_pts.push_back(car_y);
          } else {
            // use pair of points from previous path

            // append to list of anchor points
            anchor_x_pts.push_back(previous_path_x[path_size - 2]);
            anchor_x_pts.push_back(previous_path_x[path_size - 1]);

            anchor_y_pts.push_back(previous_path_y[path_size - 2]);
            anchor_y_pts.push_back(previous_path_y[path_size - 1]);
          }

          // append regularly spaced points (30 m. apart) before reference starting point
          // in Frenet coordinates:
          double target_d = 2 + our_lane * 4;  // d-coord of target lane

          vector<double> next_wp0 = getXY((car_s + SPACING), target_d, waypoints.map_s_, waypoints.map_x_,
                                          waypoints.map_y_);
          vector<double> next_wp1 = getXY((car_s + SPACING * 2), target_d, waypoints.map_s_, waypoints.map_x_,
                                          waypoints.map_y_);
          vector<double> next_wp2 = getXY((car_s + SPACING * 3), target_d, waypoints.map_s_, waypoints.map_x_,
                                          waypoints.map_y_);

          // append the next waypoints to list of anchor points
          anchor_x_pts.push_back(next_wp0[0]);
          anchor_x_pts.push_back(next_wp1[0]);
          anchor_x_pts.push_back(next_wp2[0]);

          anchor_y_pts.push_back(next_wp0[1]);
          anchor_y_pts.push_back(next_wp1[1]);
          anchor_y_pts.push_back(next_wp2[1]);

          // convert coordinates to local system
          for (int i = 0; i < anchor_x_pts.size(); i++) {
            // change our car reference angle to 0 degree
            double shift_x = anchor_x_pts[i] - ref_x;
            double shift_y = anchor_y_pts[i] - ref_y;

            // rotate
            anchor_x_pts[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            anchor_y_pts[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

          // make spline
          tk::spline s_spline;

          // set anchor points on spline
          s_spline.set_points(anchor_x_pts, anchor_y_pts);

          // append points from previous path to avoid interruption
          for (int i = 0; i < path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // goal coordinates
          // compute method to divide spline points for moving at reference speed (ref_speed)
          double goal_x = SPACING;
          double goal_y = s_spline(goal_x);

          double goal_dist = sqrt((goal_x * goal_x) + (goal_y * goal_y));

          double x_add_on = 0;

          // Populate remaining path points
          for (int i = 1; i < 50 - path_size; i++) {
            // when overly slow, then accelerate a little
            if (car_speed < ref_speed) {
              car_speed += (MS_TO_MPH / 10);     // 0.224;
            } // when overly fast, then slow down a little
            else if (car_speed > ref_speed) {
              car_speed -= (MS_TO_MPH / 10);    // 0.224;
            }
            // compute spacing of number of points dependent on wanted car speed
            double N = (goal_dist / (TIME_INTERVAL * car_speed / MS_TO_MPH)); // no of points
            double x_pnt = x_add_on + (goal_x) / N;
            // put y at the spline
            double y_pnt = s_spline(x_pnt);

            x_add_on = x_pnt;

            double x_ref = x_pnt;
            double y_ref = y_pnt;

            // convert coordinates back to normal
            x_pnt = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_pnt = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            // add reference coordinates
            x_pnt += ref_x;
            y_pnt += ref_y;

            // at last, append them to next coordinates
            next_x_vals.push_back(x_pnt);
            next_y_vals.push_back(y_pnt);
          }
// END custom path planning -----------------------------------------------------
          // 6. Send to Simulator
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

