#ifndef MANEUVER_PLANNER_H
#define MANEUVER_PLANNER_H

#include <vector>
#include "maneuver_planner_1d.h"

/**
 * @brief class for maneuvers planning
 */
class ManeuverPlanner {
 public:

   void calc_acceleration (
     std::vector<double>& next_s_vals,
     std::vector<double>& next_d_vals,
     int maneuver_min_steps_count,
     double start_s,
     double end_s_dot,
     double start_d,
     double end_d
   );

  void calc_change_lane (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    int maneuver_min_steps_count,
    double start_s,
    double start_d,
    double end_d
  );

  int get_steps_left () { return s_planner.get_steps_left(); }

  void update_maneuver(
    int step_passed,
    double current_s,
    double current_d
  );

 private:

  ManeuverPlanner1d s_planner;
  ManeuverPlanner1d d_planner;

};

#endif /* MANEUVER_PLANNER_H */
