#include "maneuver_planner.h"

void ManeuverPlanner::init_maneuver(
  double start_s
){
  maneuver_step = 0;
  maneuver_start_s = start_s;
  maneuver_current_s = start_s;
  maneuver_t = 0;

  jmt.generate_points(
    coords,
    coords_dot,
    coords_dot_dot,
    {start_s, 0, 0},
    {start_s + 50, 0.44704 * 50, 0},
    5, // 5 sec for maneuver
    manuver_steps_count,
    manuver_step_dt // dt for calc steps
  );
}

void ManeuverPlanner::update_maneuver(
  int step_passed,
  double current_s
){
  maneuver_step += step_passed;
  maneuver_current_s = current_s;
  maneuver_t += step_passed * 0.02;
}

void ManeuverPlanner::get_next_coords (std::vector<double>& coords){
  coords.resize(manuver_steps_count - maneuver_step);
  std::copy(this->coords.begin() + maneuver_step, this->coords.end(), coords.begin());
}


//
//           double T = 5;
//           int steps_processed = 250 - previous_path_x.size();
//           cout <<
//             "ms: " << manuver_step <<
//             " passed: " << manuver_passed_s <<
//             " manuver t: " << manuver_t <<
//             endl;
//
//           if (manuver_step == 0) {
//             manuver_start_s = car_s;
//             manuver_t = 0;
//           } else {
//             manuver_t += steps_processed * 0.02;
//             manuver_step += steps_processed;
//           }
//           manuver_passed_s = car_s - manuver_start_s;
//
//           vector<double> next_s_vals;
//           jmt.generate_points(
//             next_s_vals,
//             {car_s, car_speed, car_accel},
//             {car_s + 50 - manuver_passed_s, vl, 0},
//             T - manuver_t,
//             250,
//             0.02
//           );
