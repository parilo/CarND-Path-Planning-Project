#include <iostream>
#include "maneuver_planner.h"

/**
 * @brief Start new manuver with coordinate. first and second derivative will be taken from previous maneuver current point
 * @param coord
 */
void ManeuverPlanner1d::start_new_maneuver(double coord)
{
  next_maneuver_start_coord = coord;
  next_maneuver_start_coord_dot = 0;
  next_maneuver_start_coord_dot_dot = 0;

  if(coords.size() > 0 && maneuver_step < coords.size()){
    next_maneuver_start_coord_dot = coords_dot [maneuver_step];
    next_maneuver_start_coord_dot_dot = coords_dot_dot [maneuver_step];
  }

  coords.clear();
  coords_dot.clear();
  coords_dot_dot.clear();
  maneuver_step = 0;
  maneuver_t = 0;
}

/**
 * @brief get current performing point
 * @param current_coord - output coordinate
 * @param current_coord_dot - output first derivative
 * @param current_coord_dot_dot - output second derivative
 */
void ManeuverPlanner1d::get_current_coords(
  double& current_coord,
  double& current_coord_dot,
  double& current_coord_dot_dot
)
{
  current_coord = 0;
  current_coord_dot = 0;
  current_coord_dot_dot = 0;

  if(coords.size() > 0 && maneuver_step < coords.size()){
    current_coord = coords [maneuver_step];
    current_coord_dot = coords_dot [maneuver_step];
    current_coord_dot_dot = coords_dot_dot [maneuver_step];
  }
}

/**
 * @brief add change coordinate JMT maneuver
 * @param end_coord - desired end coordinate
 * @param maneuver_time
 * @return added steps
 */
int ManeuverPlanner1d::add_change_coord(
  double end_coord,
  double maneuver_time
)
{
  int maneuver_steps_count = maneuver_time / maneuver_step_dt;
  int steps = jmt.generate_points_change_coord(
    coords,
    coords_dot,
    coords_dot_dot,
    {next_maneuver_start_coord, next_maneuver_start_coord_dot, next_maneuver_start_coord_dot_dot},
    {end_coord, 0, 0},
    maneuver_time,
    maneuver_steps_count,
    maneuver_step_dt // dt for calc steps
  );

  if (steps > 0)
  {
    next_maneuver_start_coord = coords.back();
    next_maneuver_start_coord_dot = coords_dot.back();
    next_maneuver_start_coord_dot_dot = coords_dot_dot.back();
  }

  return steps;
}

/**
 * @brief add constant coordinate trajectory for given amount of steps
 * @param steps_num
 * @return added steps
 */
int ManeuverPlanner1d::add_constant_coord(
  double steps_num
)
{
  for(int i=0; i<steps_num; i++)
  {
    coords.push_back(next_maneuver_start_coord);
    coords_dot.push_back(0);
    coords_dot_dot.push_back(0);
  }

  return steps_num;
}

/**
 * @brief add acceleration maneuver
 * @param end_coord_dot - desired end velocity
 * @param max_steps_count - maximum calculated steps
 * @return added steps
 */
int ManeuverPlanner1d::add_acceleration(
  double end_coord_dot,
  int max_steps_count
)
{

  int steps = jmt.generate_points_accelerate(
    coords,
    coords_dot,
    coords_dot_dot,
    next_maneuver_start_coord,
    next_maneuver_start_coord_dot,
    next_maneuver_start_coord_dot_dot,
    end_coord_dot,
    0,
    maneuver_step_dt,
    max_steps_count
  );

  if (steps > 0)
  {
    next_maneuver_start_coord = coords.back();
    next_maneuver_start_coord_dot = coords_dot.back();
    next_maneuver_start_coord_dot_dot = coords_dot_dot.back();
  }

  return steps;
}

/**
 * @brief add constant speed maneuver for given amount of steps
 * @param steps_num
 * @return added steps
 */
int ManeuverPlanner1d::add_constant_speed(
  double steps_num
)
{
  for(int i=0; i<steps_num; i++)
  {
    coords.push_back(next_maneuver_start_coord);
    coords_dot.push_back(next_maneuver_start_coord_dot);
    coords_dot_dot.push_back(0);
    next_maneuver_start_coord += next_maneuver_start_coord_dot * maneuver_step_dt;
  }

  return steps_num;
}

/**
 * @brief updates current performing point of this maneuver
 * @param step_passed - steps passed since last update
 */
void ManeuverPlanner1d::update_maneuver(
  int step_passed
){
  maneuver_step += step_passed;
  maneuver_t += step_passed * 0.02;
}

/**
 * @brief get coordinates from current performing point
 * @param steps_count
 * @param coords - output coordinates
 */
void ManeuverPlanner1d::get_next_coords (
  int steps_count,
  std::vector<double>& coords
)
{
  coords.resize(steps_count);
  std::copy(this->coords.begin() + maneuver_step, this->coords.begin() + steps_count, coords.begin());
}
