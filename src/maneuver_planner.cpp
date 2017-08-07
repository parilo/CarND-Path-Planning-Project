#include <iostream>
#include "maneuver_planner.h"

/**
 * @brief Calculate acceleration or deceleration in lane maneuver.
 *   It is combination of acceleration in s and slightly change position in d
 * @param next_s_vals - output s coordinates
 * @param next_d_vals - output d coordinates
 * @param maneuver_min_steps_count - minimum steps count to generate
 * @param start_s - start s coordinate
 * @param end_s_dot - desired end maneuver s derivative
 * @param start_d - start d coordinate
 * @param end_d - desired end maneuver d coordinate
 */
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

  int s_steps = 0;
  s_planner.start_new_maneuver(start_s);
  // add acceleration
  s_steps += s_planner.add_acceleration(end_s_dot, maneuver_min_steps_count);
  if (s_steps < maneuver_min_steps_count)
  {
    // add constant speed if needed after acceleration
    s_steps += s_planner.add_constant_speed(maneuver_min_steps_count - s_steps);
  }
  s_planner.get_next_coords(maneuver_min_steps_count, next_s_vals);

  // d part of maneuver
  // time to perform d adjust
  double d_maneuver_time = fabs(start_d - end_d) * 2.0;
  int d_steps = 0;
  d_planner.start_new_maneuver(start_d);
  if (current_s_dot > 10.0){
    // don't want to andjust d on small s velocity
    d_steps += d_planner.add_change_coord(end_d, d_maneuver_time);
  }
  if (d_steps < maneuver_min_steps_count)
  {
    // add constant coord if needed
    d_steps += d_planner.add_constant_coord(maneuver_min_steps_count - d_steps);
  }
  d_planner.get_next_coords(maneuver_min_steps_count, next_d_vals);

}

/**
 * @brief update maneuver current performing step
 * @param step_passed
 */
void ManeuverPlanner::update_maneuver(
  int step_passed
)
{
  s_planner.update_maneuver (step_passed);
  d_planner.update_maneuver (step_passed);
}

/**
 * @brief Calculate change lane maneuver.
 *   It is keep velocity in s and significantly change position in d
 * @param next_s_vals - output s coordinates
 * @param next_d_vals - output d coordinates
 * @param maneuver_min_steps_count - minimum steps count to generate
 * @param start_s - start s coordinate
 * @param start_d - start d coordinate
 * @param end_d - desired end maneuver d coordinate
 */
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

  // time to perform change lane
  double d_maneuver_time = fabs(start_d - end_d) / 2.0;
  int d_steps = 0;
  d_planner.start_new_maneuver(start_d);
  d_steps += d_planner.add_change_coord(end_d, d_maneuver_time);
  if (d_steps < maneuver_min_steps_count)
  {
    // add constant coord if needed
    d_steps += d_planner.add_constant_coord(maneuver_min_steps_count - d_steps);
  }
  d_planner.get_next_coords(maneuver_min_steps_count, next_d_vals);
}
