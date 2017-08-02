#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "jmt.h"
#include "maneuver_planner.h"
#include "map_funcs.h"
#include "trajectory_smoother.h"

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

  ManeuverPlanner maneuver_planner;

  h.onMessage([
    &map_waypoints_x,
    &map_waypoints_y,
    &map_waypoints_s,
    &map_waypoints_dx,
    &map_waypoints_dy,
    // &jmt,
    // &prev_car_speed,
    &maneuver_planner
    ] (
      uWS::WebSocket<uWS::SERVER> ws,
      char *data,
      size_t length,
      uWS::OpCode opCode
    ) {
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
            // cout << "json: " << j.dump();

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
            double car_speed = double(j[1]["speed"]); //convert to meters per second

          	// Previous path data given to the Planner
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // for (auto& other_car : sensor_fusion) {
            //   cout <<
            //     "  id: " << other_car[0] <<
            //     "  s: " << other_car[5] <<
            //     "  d: " << other_car[6] <<
            //     endl;
            // }

            json msgJson;

            vector<double> next_s_vals;
            vector<double> next_x_vals;
          	vector<double> next_y_vals;

            int passed_steps = maneuver_planner.get_steps_left() - previous_path_x.size();
            if (previous_path_x.size() < 150) {

              maneuver_planner.init_maneuver(car_s);
              maneuver_planner.get_next_coords(next_s_vals);
              double px=0, py=0;
              vector<double> new_xs, new_ys;
              for (int i=0; i<next_s_vals.size(); i++) {
                vector<double> xy = getXYSplined(next_s_vals[i], car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                new_xs.push_back(xy[0]);
                new_ys.push_back(xy[1]);
              }

              TrajectorySmoother::merge_trajectoies(next_x_vals, previous_path_x, new_xs, 100);
              TrajectorySmoother::smooth_trajectory(next_x_vals);
              TrajectorySmoother::smooth_trajectory(next_x_vals);
              TrajectorySmoother::merge_trajectoies(next_y_vals, previous_path_y, new_ys, 100);
              TrajectorySmoother::smooth_trajectory(next_y_vals);
              TrajectorySmoother::smooth_trajectory(next_y_vals);
            } else {
              maneuver_planner.update_maneuver(
                passed_steps,
                car_s
              );
              next_x_vals.resize(previous_path_x.size());
              next_y_vals.resize(previous_path_y.size());
              copy(previous_path_x.begin(), previous_path_x.end(), next_x_vals.begin());
              copy(previous_path_y.begin(), previous_path_y.end(), next_y_vals.begin());
            }

            // double T = 5;
            cout <<
              "car: s: " << car_s <<
              " d: " << car_d <<
              " v: " << car_speed <<
              // " a: " << car_accel <<
              " prev path: " << previous_path_x.size() <<
              " m s: " << maneuver_planner.get_step() <<
              " passed steps: " << passed_steps <<
              " passed s: " << maneuver_planner.get_passed_s() <<
              " manuver t: " << maneuver_planner.get_t() <<
              " steps left: " << maneuver_planner.get_steps_left() <<
              endl;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

            // cout << "msg: " << msgJson.dump() << endl;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
