#include <iostream>
#include "maneuver_planner.h"

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
//  maneuver_current_coord = coord;
  maneuver_t = 0;
}

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

// void ManeuverPlanner1d::init_start_params_from_current_maneuver(double& start_v, double& start_a)
// {
//   start_v = 0;
//   start_a = 0;
//   if(coords.size() > 0 && maneuver_step < maneuver_steps_count){
//     start_v = coords_dot [maneuver_step];
//     start_a = coords_dot_dot [maneuver_step];
//   }
// }

int ManeuverPlanner1d::add_change_coord(
  double end_coord,
  double maneuver_time
)
{
  int maneuver_steps_count = maneuver_time / maneuver_step_dt;
std::cout << "change coord: steps: " << maneuver_steps_count << " t: " << maneuver_time << std::endl;
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

//  int steps = jmt.generate_points_change_coord(
//    coords,
//    coords_dot,
//    coords_dot_dot,
//    next_maneuver_start_coord, next_maneuver_start_coord_dot, next_maneuver_start_coord_dot_dot,
//    end_coord, 0, 0,
//    maneuver_step_dt, // dt for calc steps
//    maneuver_steps_count
//  );

  if (steps > 0)
  {
    next_maneuver_start_coord = coords.back();
    next_maneuver_start_coord_dot = coords_dot.back();
    next_maneuver_start_coord_dot_dot = coords_dot_dot.back();
  }

  return steps;
}

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

int ManeuverPlanner1d::add_acceleration(
//  double end_coord,
  double end_coord_dot,
//  double maneuver_time
  int max_steps_count
)
{
//  int maneuver_steps_count = maneuver_time / maneuver_step_dt;
//  jmt.generate_points(
//    coords,
//    coords_dot,
//    coords_dot_dot,
//    {next_maneuver_start_coord, next_maneuver_start_coord_dot, next_maneuver_start_coord_dot_dot},
//    {end_coord, end_coord_dot, 0},
//    maneuver_time,
//    maneuver_steps_count,
//    maneuver_step_dt // dt for calc steps
//  );

//  int maneuver_steps_count = jmt.generate_points(
//    coords,
//    coords_dot,
//    coords_dot_dot,
//    next_maneuver_start_coord,
//    next_maneuver_start_coord_dot,
//    next_maneuver_start_coord_dot_dot,
//    end_coord_dot,
//    0,
//    max_steps_count,
//    maneuver_step_dt // dt for calc steps
//  );

//  std::vector<double> p;
//  jmt.generate_parameters_accelerate(
//    p,
//    next_maneuver_start_coord,
//    next_maneuver_start_coord_dot,
//    next_maneuver_start_coord_dot_dot,
//    end_coord_dot,
//    0,
//    maneuver_step_dt
//  );

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

//  std::vector<double>& coords,
//  std::vector<double>& coords_dot,
//  std::vector<double>& coords_dot_dot,
//  double start,
//  double start_v,
//  double start_a,
//  double end_v,
//  double end_a,
//  double dt,
//  int steps_count


//  next_maneuver_start_coord = end_coord;
  if (steps > 0)
  {
    next_maneuver_start_coord = coords.back();
    next_maneuver_start_coord_dot = coords_dot.back();
    next_maneuver_start_coord_dot_dot = coords_dot_dot.back();
  }

  return steps; //maneuver_steps_count;
}

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

void ManeuverPlanner1d::update_maneuver(
  int step_passed,
  double current_coord
){
  maneuver_step += step_passed;
//  maneuver_current_coord = current_coord;
  maneuver_t += step_passed * 0.02;
}

void ManeuverPlanner1d::get_next_coords (
  int steps_count,
  std::vector<double>& coords
)
{
  coords.resize(steps_count);
  std::copy(this->coords.begin(), this->coords.begin() + steps_count, coords.begin());
//  coords.resize(this->coords.size() - maneuver_step);
//  std::copy(this->coords.begin() + maneuver_step, this->coords.end(), coords.begin());
}
