#ifndef BEHAVOR_LAYER_H
#define BEHAVOR_LAYER_H

#include <vector>
#include "maneuver_planner.h"

/**
 * @brief class for generation Jerk Minimizing Trajectory
 */
class BehaviorLayer {
 public:

  enum class CarState {START, MOVE_FORWARD, FOLLOW, CHANGING_LEFT, CHANGING_RIGHT};

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

  const double safety_time_to_collision = 10;
  const double safety_change_lane_gap = 8;
  const double safety_front_car_dist = 20;
  const int maneuver_min_steps_count = 150;
  const int maneuver_recalc_steps_count = 100;

  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;

  CarState current_state = CarState::START;
  int dst_lane_index = 1;
  ManeuverPlanner maneuver_planner;

  bool update_current_state (
    const std::vector<double>& car_state, // x, y, s, d, yaw, v
    const std::vector<std::vector<double>>& sensor_data
  );

  int try_change_lane (
    const std::vector<double>& car_state,
    const std::vector<double>& lanes_time_to_collision,
    const std::vector<double>& lanes_distances,
    const std::vector<double>& lanes_speed,
    const std::vector<std::vector<double>>& sensor_data
  );

  bool is_dst_lane_reached (const std::vector<double>& car_state);

  double get_front_car_speed (
    const std::vector<double>& car_state,
    const std::vector<std::vector<double>>& sensor_data
  );

  bool is_lane_closed (
    int lane_index,
    const std::vector<double>& car_state,
    const std::vector<std::vector<double>>& sensor_data
  );

  void calc_move_forward (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    const std::vector<double>& car_state
  );

  void calc_move_forward (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    const std::vector<double>& car_state,
    double dst_speed
  );

  void calc_change_lane (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    const std::vector<double>& car_state,
    int new_lane_index
  );

};

#endif /* BEHAVOR_LAYER_H */
