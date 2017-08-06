#ifndef MANEUVER_PLANNER_1D_H
#define MANEUVER_PLANNER_1D_H

#include <vector>
#include "jmt.h"

/**
 * @brief class for maneuvers planning, holds currect car coord state and use it
 *   for the further planning
 */
class ManeuverPlanner1d {
 public:

  void start_new_maneuver(double coord);

  int add_change_coord(
    double end_coord,
    double maneuver_time
  );

  int add_constant_coord(
    double steps_num
  );

  int add_acceleration(
  //  double end_coord,
    double end_coord_dot,
  //  double maneuver_time
    int max_steps_count
  );

  int add_constant_speed(
    double steps_num
  );

  void update_maneuver(
    int step_passed,
    double current_coord
  );

  void get_current_coords(
    double& current_coord,
    double& current_coord_dot,
    double& current_coord_dot_dot
  );

  void get_next_coords (
    int steps_count,
    std::vector<double>& coords
  );

  int get_steps_left () { return coords.size() - maneuver_step; }

 private:

  const double maneuver_step_dt = 0.02;

  int maneuver_step = 0;
//  double maneuver_current_coord = 0;
  double maneuver_t = 0;

  double next_maneuver_start_coord;
  double next_maneuver_start_coord_dot;
  double next_maneuver_start_coord_dot_dot;

  JMT jmt;

  std::vector<double> coords;
  std::vector<double> coords_dot;
  std::vector<double> coords_dot_dot;

  // void init_start_params_from_current_maneuver (double& start_v, double& start_a);

};

#endif /* MANEUVER_PLANNER_H */
