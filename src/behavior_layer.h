#ifndef BEHAVOR_LAYER_H
#define BEHAVOR_LAYER_H

#include <vector>
#include "maneuver_planner.h"

/**
 * @brief class for generation Jerk Minimizing Trajectory
 */
class BehaviorLayer {
 public:

  enum class CarState {MOVE_FORWARD, FOLLOW, CHANGING_LEFT, CHANGING_RIGHT};

  void set_map_waypoints (
   const std::vector<double>& map_waypoints_s,
   const std::vector<double>& map_waypoints_x,
   const std::vector<double>& map_waypoints_y
  );

  void process_step (
   std::vector<double>& next_x_vals,
   std::vector<double>& next_y_vals,
   const std::vector<double>& car_state, // x, y, s, d, yaw, v
   const std::vector<double>& previous_path_x,
   const std::vector<double>& previous_path_y,
   const std::vector<std::vector<double>>& sensor_data
  );

 private:

  const double forward_until_time = 5;

  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;

  CarState current_state = CarState::MOVE_FORWARD;
  int dst_lane_index = 1;
  ManeuverPlanner maneuver_planner;

  bool update_current_state (
    const std::vector<double>& car_state,
    const std::vector<std::vector<double>>& sensor_data
  );

  int try_change_lane (
    const std::vector<double>& car_state,
    const std::vector<double>& lanes_values
  );

  bool is_dst_lane_reached (const std::vector<double>& car_state);

};

#endif /* BEHAVOR_LAYER_H */
