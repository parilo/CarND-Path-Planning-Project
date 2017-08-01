#ifndef MANEUVER_PLANNER_H
#define MANEUVER_PLANNER_H

#include <vector>
#include "jmt.h"

/**
 * @brief class for maneuvers planning
 */
class ManeuverPlanner {
 public:

  void init_maneuver(
    double start_s
  );

  void update_maneuver(
    int step_passed,
    double current_s
  );

  int get_step () { return maneuver_step; }
  int get_steps_left () { return maneuver_steps_count - maneuver_step; }
  double get_passed_s () { return maneuver_current_s - maneuver_start_s; }
  double get_t () { return maneuver_t; }

  void get_next_coords (std::vector<double>& coords);

 private:

  const int maneuver_steps_count = 200;
  const double maneuver_step_dt = 0.02;

  int maneuver_step = 0;
  double maneuver_start_s = 0;
  double maneuver_current_s = 0;
  double maneuver_t = 0;

  JMT jmt;

  std::vector<double> coords;
  std::vector<double> coords_dot;
  std::vector<double> coords_dot_dot;

};

#endif /* MANEUVER_PLANNER_H */
