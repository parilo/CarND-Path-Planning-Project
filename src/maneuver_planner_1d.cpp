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
  maneuver_current_coord = coord;
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
  jmt.generate_points(
    coords,
    coords_dot,
    coords_dot_dot,
    {next_maneuver_start_coord, next_maneuver_start_coord_dot, next_maneuver_start_coord_dot_dot},
    {end_coord, 0, 0},
    maneuver_time,
    maneuver_steps_count,
    maneuver_step_dt // dt for calc steps
  );

  next_maneuver_start_coord = end_coord;
  next_maneuver_start_coord_dot = 0;
  next_maneuver_start_coord_dot_dot = 0;

  return maneuver_steps_count;
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
  double end_coord,
  double end_coord_dot,
  double maneuver_time
)
{
  int maneuver_steps_count = maneuver_time / maneuver_step_dt;
  jmt.generate_points(
    coords,
    coords_dot,
    coords_dot_dot,
    {next_maneuver_start_coord, next_maneuver_start_coord_dot, next_maneuver_start_coord_dot_dot},
    {end_coord, end_coord_dot, 0},
    maneuver_time,
    maneuver_steps_count,
    maneuver_step_dt // dt for calc steps
  );

  next_maneuver_start_coord = end_coord;
  next_maneuver_start_coord_dot = end_coord_dot;
  next_maneuver_start_coord_dot_dot = 0;

  return maneuver_steps_count;
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
  maneuver_current_coord = current_coord;
  maneuver_t += step_passed * 0.02;
}

void ManeuverPlanner1d::get_next_coords (std::vector<double>& coords){
  coords.resize(this->coords.size() - maneuver_step);
  std::copy(this->coords.begin() + maneuver_step, this->coords.end(), coords.begin());
}












// void ManeuverPlanner1d::calc_maneuver(
//   double start_coord
// ){
//   double start_v = 0;
//   double start_a = 0;
//   if(coords.size() > 0 && maneuver_step < maneuver_steps_count){
//     start_v = coords_dot [maneuver_step];
//     start_a = coords_dot_dot [maneuver_step];
// //    std::cout << "new maneuver: v: " << start_v << " a: " << start_a << std::endl;
//   }
//
//   maneuver_step = 0;
//   maneuver_start_coord = start_coord;
//   maneuver_current_coord = start_coord;
//   maneuver_t = 0;
//
//   double end_t = 5; // 5 sec for maneuver
//   double end_v = 0.44704 * 48;
//   double end_coord = start_coord + (start_v + end_v) * 0.5 * end_t;
//
//   jmt.generate_points(
//     coords,
//     coords_dot,
//     coords_dot_dot,
//     {start_coord, start_v, start_a},
//     {end_coord, end_v, 0},
//     end_t,
//     maneuver_steps_count,
//     maneuver_step_dt // dt for calc steps
//   );
//
// //  for(int i=0; i<coords.size(); i++) {
// //    std::cout <<
// //      "m i: " << i <<
// //      " x: " << coords [i] <<
// //      " v: " << coords_dot [i] <<
// //      " a: " << coords_dot_dot [i] <<
// //      std::endl;
// //  }
//
// }
