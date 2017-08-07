#ifndef BEHAVOR_LAYER_H
#define BEHAVOR_LAYER_H

#include <vector>
#include "maneuver_planner.h"

/**
 * @brief Behavioral layer for performing and desision making about maneuvers
 */
class BehaviorLayer {
 public:

  enum class CarState {START, MOVE_FORWARD, FOLLOW, CHANGING_LEFT, CHANGING_RIGHT};

  /**
   * @brief set map keypoints
   * @param map_waypoints_s
   * @param map_waypoints_x
   * @param map_waypoints_y
   */
  void set_map_waypoints (
   const std::vector<double>& map_waypoints_s,
   const std::vector<double>& map_waypoints_x,
   const std::vector<double>& map_waypoints_y
  );

  /**
   * @brief process input from simulator
   * @param next_x_vals
   * @param next_y_vals
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param previous_path_x
   * @param previous_path_y
   * @param sensor_data - information about other cars on the road
   */
  void process_step (
   std::vector<double>& next_x_vals,
   std::vector<double>& next_y_vals,
   const std::vector<double>& car_state, // x, y, s, d, yaw, v
   const std::vector<double>& previous_path_x,
   const std::vector<double>& previous_path_y,
   const std::vector<std::vector<double>>& sensor_data
  );

 private:

  // need to slow avoiding collisions
  const double safety_time_to_collision = 10; // secs

  // needed space between cars for lane changing
  const double safety_change_lane_gap = 10; // meters

  // distance of the front car for following
  const double safety_front_car_dist = 25; // meters

  // future trajectory horizon params
  const int maneuver_min_steps_count = 150; // one step is 0.02 sec
  const int maneuver_recalc_steps_count = 140;

  //maximum forward speed
  double forward_speed = 0.44704 * 46; // m/s

  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;

  CarState current_state = CarState::START;
  int dst_lane_index = 1;
  ManeuverPlanner maneuver_planner;

  /**
   * @brief Central desision making method.
   *   Desides wether it is needed to change current state
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param sensor_data - information about other cars on the road
   * @return
   */
  bool update_current_state (
    const std::vector<double>& car_state, // x, y, s, d, yaw, v
    const std::vector<std::vector<double>>& sensor_data
  );

  /**
   * @brief Desides wether it is reasonable to change lane
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param lanes_time_to_collision - for every lane
   * @param lanes_distances - for every lane
   * @param lanes_speed - for every lane
   * @param sensor_data - information about other cars on the road
   * @return
   */
  int try_change_lane (
    const std::vector<double>& car_state,
    const std::vector<double>& lanes_time_to_collision,
    const std::vector<double>& lanes_distances,
    const std::vector<double>& lanes_speed,
    const std::vector<std::vector<double>>& sensor_data
  );

  /**
   * @brief is destination lane reached
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @return
   */
  bool is_dst_lane_reached (const std::vector<double>& car_state);

  /**
   * @brief get front car speed
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param sensor_data - information about other cars on the road
   * @return
   */
  double get_front_car_speed (
    const std::vector<double>& car_state,
    const std::vector<std::vector<double>>& sensor_data
  );

  /**
   * @brief is lane is closed to be changed in. Other car may interfere
   * @param lane_index
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param sensor_data
   * @return
   */
  bool is_lane_closed (
    int lane_index,
    const std::vector<double>& car_state,
    const std::vector<std::vector<double>>& sensor_data
  );

  /**
   * @brief Calculate move forward in lane trajectory
   * @param next_s_vals - output trajectory s values
   * @param next_d_vals - output trajectory d values
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param dst_speed
   */
  void calc_move_forward (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    const std::vector<double>& car_state,
    double dst_speed
  );

  /**
   * @brief Calculate change lane trajectory
   * @param next_s_vals - output trajectory s values
   * @param next_d_vals - output trajectory d values
   * @param car_state - vector [x, y, s, d, yaw, v]
   * @param new_lane_index
   */
  void calc_change_lane (
    std::vector<double>& next_s_vals,
    std::vector<double>& next_d_vals,
    const std::vector<double>& car_state,
    int new_lane_index
  );

};

#endif /* BEHAVOR_LAYER_H */
