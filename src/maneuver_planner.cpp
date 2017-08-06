#include <iostream>
#include "maneuver_planner.h"

void ManeuverPlanner::calc_acceleration (
  std::vector<double>& next_s_vals,
  std::vector<double>& next_d_vals,
  int maneuver_min_steps_count,
  double start_s,
  double end_s_dot,
  double start_d,
  double end_d
)
{
  // s part of maneuver
  double current_s, current_s_dot, current_s_dot_dot;
  s_planner.get_current_coords(current_s, current_s_dot, current_s_dot_dot);
//  double s_maneuver_time = fabs(current_s_dot - end_s_dot) / 5.2;
//  double s_maneuver_time = fabs(current_s_dot - end_s_dot) / 2.5;
//  double end_s = start_s + (current_s_dot + end_s_dot) * 0.5 * s_maneuver_time;
//  double end_s = start_s + (current_s_dot * s_maneuver_time + 5.0 * s_maneuver_time * s_maneuver_time);
// std::cout << " accel time: " << s_maneuver_time << " accel ds: " << end_s - start_s << std::endl;

  int s_steps = 0;
  s_planner.start_new_maneuver(start_s);
//  if (s_maneuver_time > 0)
  s_steps += s_planner.add_acceleration(end_s_dot, maneuver_min_steps_count);
//    s_steps += s_planner.add_acceleration(end_s, end_s_dot, s_maneuver_time);
std::cout << "accel steps: " << s_steps  << std::endl;
  if (s_steps < maneuver_min_steps_count)
  {
    s_steps += s_planner.add_constant_speed(maneuver_min_steps_count - s_steps);
  }
  s_planner.get_next_coords(maneuver_min_steps_count, next_s_vals);
//  int s_maneuver_size = std::max(s_steps, maneuver_min_steps_count);

  // d part of maneuver
  double d_maneuver_time = fabs(start_d - end_d) * 2.0;
std::cout << "start_d: " << start_d << " end_d: " << end_d << " t: " << d_maneuver_time << std::endl;
//  double d_maneuver_time = 2.0;
  int d_steps = 0;
  d_planner.start_new_maneuver(start_d);
  if (current_s_dot > 10.0){
  d_steps += d_planner.add_change_coord(end_d, d_maneuver_time);
std::cout << "d steps: " << d_steps << " t: " << d_maneuver_time << std::endl;
  }
  if (d_steps < maneuver_min_steps_count)
  {
    d_steps += d_planner.add_constant_coord(maneuver_min_steps_count - d_steps);
  }
  d_planner.get_next_coords(maneuver_min_steps_count, next_d_vals);

std::cout << "ns: " << next_s_vals.size() << " nd: " << next_d_vals.size() << std::endl;
}

void ManeuverPlanner::update_maneuver(
  int step_passed,
  double current_s,
  double current_d
)
{
  s_planner.update_maneuver (step_passed, current_s);
  d_planner.update_maneuver (step_passed, current_d);
}

void ManeuverPlanner::calc_change_lane (
  std::vector<double>& next_s_vals,
  std::vector<double>& next_d_vals,
  int maneuver_min_steps_count,
  double start_s,
  double start_d,
  double end_d
)
{
  s_planner.start_new_maneuver(start_s);
  s_planner.add_constant_speed(maneuver_min_steps_count);
  s_planner.get_next_coords(maneuver_min_steps_count, next_s_vals);

  double d_maneuver_time = fabs(start_d - end_d) / 2.0;
  int d_steps = 0;
  d_planner.start_new_maneuver(start_d);
  d_steps += d_planner.add_change_coord(end_d, d_maneuver_time);
  if (d_steps < maneuver_min_steps_count)
  {
    d_steps += d_planner.add_constant_coord(maneuver_min_steps_count - d_steps);
  }
  d_planner.get_next_coords(maneuver_min_steps_count, next_d_vals);
}
