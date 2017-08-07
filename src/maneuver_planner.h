#ifndef MANEUVER_PLANNER_H
#define MANEUVER_PLANNER_H

#include <vector>
#include "maneuver_planner_1d.h"

/**
 * @brief Planner for 2d maneuvers in Frenet road frame (s, d)
 *   s - along road
 *   d - perpendicular road
 *  Also keeps and updates current performing maneuver step to make smooth transitions between maneuvers
 */
class ManeuverPlanner {
 public:

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
  void calc_acceleration (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    int maneuver_min_steps_count,
    double start_s,
    double end_s_dot,
    double start_d,
    double end_d
  );

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
  void calc_change_lane (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    int maneuver_min_steps_count,
    double start_s,
    double start_d,
    double end_d
  );

  /**
   * @brief Get left steps to perform
   * @return
   */
  int get_steps_left () { return s_planner.get_steps_left(); }

  /**
   * @brief update maneuver current performing step
   * @param step_passed
   */
  void update_maneuver(
    int step_passed
  );

 private:

  ManeuverPlanner1d s_planner;
  ManeuverPlanner1d d_planner;

};

#endif /* MANEUVER_PLANNER_H */
