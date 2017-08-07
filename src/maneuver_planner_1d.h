#ifndef MANEUVER_PLANNER_1D_H
#define MANEUVER_PLANNER_1D_H

#include <vector>
#include "jmt.h"

/**
 * @brief Planner of 1d JMT maneuvers, it allow stack different trajectories on one maneuver. Also
 *   stores and updates maneuver performing state point for reiniting new maneuver from the middle point of ols maneuver
 */
class ManeuverPlanner1d {
 public:

  /**
   * @brief Start new manuver with coordinate. first and second derivative will be taken from previous maneuver current point
   * @param coord
   */
  void start_new_maneuver(double coord);

  /**
   * @brief add change coordinate JMT maneuver
   * @param end_coord - desired end coordinate
   * @param maneuver_time
   * @return added steps
   */
  int add_change_coord(
    double end_coord,
    double maneuver_time
  );

  /**
   * @brief add constant coordinate trajectory for given amount of steps
   * @param steps_num
   * @return added steps
   */
  int add_constant_coord(
    double steps_num
  );

  /**
   * @brief add acceleration maneuver
   * @param end_coord_dot - desired end velocity
   * @param max_steps_count - maximum calculated steps
   * @return added steps
   */
  int add_acceleration(
    double end_coord_dot,
    int max_steps_count
  );

  /**
   * @brief add constant speed maneuver for given amount of steps
   * @param steps_num
   * @return added steps
   */
  int add_constant_speed(
    double steps_num
  );

  /**
   * @brief updates current performing point of this maneuver
   * @param step_passed - steps passed since last update
   */
  void update_maneuver(
    int step_passed
  );

  /**
   * @brief get current performing point
   * @param current_coord - output coordinate
   * @param current_coord_dot - output first derivative
   * @param current_coord_dot_dot - output second derivative
   */
  void get_current_coords(
    double& current_coord,
    double& current_coord_dot,
    double& current_coord_dot_dot
  );

  /**
   * @brief get coordinates from current performing point
   * @param steps_count
   * @param coords - output coordinates
   */
  void get_next_coords (
    int steps_count,
    std::vector<double>& coords
  );

  /**
   * @brief get steps left to perform in current maneuver
   * @return
   */
  int get_steps_left () { return coords.size() - maneuver_step; }

 private:

  const double maneuver_step_dt = 0.02;

  int maneuver_step = 0;
  double maneuver_t = 0;

  double next_maneuver_start_coord;
  double next_maneuver_start_coord_dot;
  double next_maneuver_start_coord_dot_dot;

  JMT jmt;

  std::vector<double> coords;
  std::vector<double> coords_dot;
  std::vector<double> coords_dot_dot;

};

#endif /* MANEUVER_PLANNER_H */
