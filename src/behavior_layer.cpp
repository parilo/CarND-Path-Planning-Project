#include <iostream>
#include "behavior_layer.h"
#include "map_funcs.h"
#include "trajectory_smoother.h"

/**
 * @brief set map keypoints
 * @param map_waypoints_s
 * @param map_waypoints_x
 * @param map_waypoints_y
 */
void BehaviorLayer::set_map_waypoints (
  const std::vector<double>& map_waypoints_s,
  const std::vector<double>& map_waypoints_x,
  const std::vector<double>& map_waypoints_y
)
{
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
}

int get_lane_index (double car_d)
{
  // lanes centers positions in d are: 1.8, 5.8, 9.8 in average
  return int(round((car_d - 1.8) / 4.0));
}

double get_lane_d (int lane_index)
{
  return lane_index * 4 + 1.8;
}

/**
 * @brief Calculate cost functions for lanes
 * @param lanes_time_to_collision - for every lane
 * @param lanes_distances - for every lane
 * @param lanes_speed - for every lane
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param sensor_data - information about other cars on the road
 */
void get_lanes_values (
  std::vector<double>& lanes_time_to_collision,
  std::vector<double>& lanes_distances,
  std::vector<double>& lanes_speed,
  const std::vector<double>& car_state,
  const std::vector<std::vector<double>>& sensor_data
)
{
  lanes_time_to_collision.resize(3, 1000);
  lanes_distances.resize(3, 1000);
  lanes_speed.resize(3, 1000);
  double car_s = car_state[2];
  double car_s_dot = car_state[5];
  int my_lane_index = get_lane_index (car_state[3]);

  // inspecting information about other cars on the road
  for (auto& other_car_data : sensor_data)
  {
    // car id, x, y, vx, vy, s, d
    double other_car_vx = other_car_data[3];
    double other_car_vy = other_car_data[4];
    double other_car_s = other_car_data[5];
    double other_car_d = other_car_data[6];

    // determine lane of the car
    int lane_index = get_lane_index (other_car_d);
    // determine ds/dt, suppose car travels along the road
    double other_car_s_dot = distance (0, 0, other_car_vx, other_car_vy);

    if (
        other_car_s_dot != car_s_dot &&
        (
          (car_s < other_car_s && my_lane_index == lane_index) ||
          my_lane_index != lane_index
        )
    ) {
      double time_to_collision = (car_s - other_car_s) / (other_car_s_dot - car_s_dot);
      if (
        time_to_collision >= 0 &&
        lanes_time_to_collision[lane_index] > time_to_collision
      ) {
        lanes_time_to_collision[lane_index] = time_to_collision;
      }
    }

    double other_car_dist = other_car_s - car_s;

    // considering the closest front car
    if (
      other_car_dist >= 0 &&
      lanes_distances[lane_index] > other_car_dist
    ) {
      lanes_distances[lane_index] = other_car_dist;
      lanes_speed[lane_index] = other_car_s_dot;
    }
  }
}

/**
 * @brief get front car speed
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param sensor_data - information about other cars on the road
 * @return
 */
double BehaviorLayer::get_front_car_speed (
  const std::vector<double>& car_state,
  const std::vector<std::vector<double>>& sensor_data
)
{
  double car_s = car_state[2];
  double car_d = car_state[3];
  int my_lane_index = get_lane_index (car_d);
  double closest_car_dist = 5000;
  double closest_car_v = 1000;
  for (auto& other_car_data : sensor_data)
  {
    // car id, x, y, vx, vy, s, d
    double other_car_vx = other_car_data[3];
    double other_car_vy = other_car_data[4];
    double other_car_s = other_car_data[5];
    double other_car_d = other_car_data[6];

    // determine lane of the car
    int lane_index = get_lane_index (other_car_d);
    if (lane_index == my_lane_index)
    {
      double dist = other_car_s - car_s;
      if (dist > 0 && dist < closest_car_dist)
      {
        closest_car_dist = dist;
        closest_car_v = distance (0, 0, other_car_vx, other_car_vy);
      }
    }
  }

  return closest_car_v;
}

/**
 * @brief is lane is closed to be changed in. Other car may interfere
 * @param lane_index
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param sensor_data
 * @return
 */
bool BehaviorLayer::is_lane_closed (
  int lane_index,
  const std::vector<double>& car_state,
  const std::vector<std::vector<double>>& sensor_data
)
{
  double car_s = car_state[2];
  for (auto& other_car_data : sensor_data)
  {
    // car id, x, y, vx, vy, s, d
    double other_car_vx = other_car_data[3];
    double other_car_vy = other_car_data[4];
    double other_car_s = other_car_data[5];
    double other_car_d = other_car_data[6];

    // determine lane of the car
    int car_lane_index = get_lane_index (other_car_d);
    if (lane_index == car_lane_index)
    {
      if (fabs(other_car_s - car_s) < safety_change_lane_gap) return true;
    }
  }

  return false;
}

/**
 * @brief Desides wether it is reasonable to change lane
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param lanes_time_to_collision - for every lane
 * @param lanes_distances - for every lane
 * @param lanes_speed - for every lane
 * @param sensor_data - information about other cars on the road
 * @return
 */
int BehaviorLayer::try_change_lane (
  const std::vector<double>& car_state,
  const std::vector<double>& lanes_time_to_collision,
  const std::vector<double>& lanes_distances,
  const std::vector<double>& lanes_speed,
  const std::vector<std::vector<double>>& sensor_data
)
{
  double car_d = car_state[3];
  int my_lane_index = get_lane_index (car_d);

  // left lane
  if (my_lane_index == 0)
  {
    bool lane_open = !is_lane_closed(1, car_state, sensor_data);
    // center lane is preffered for us
    if (
      lanes_time_to_collision[1] > safety_time_to_collision &&
      lanes_distances[1] > lanes_distances[0] &&
      lane_open
    ){
      return 1;
    } else {
      return 0;
    }
  }
  // center lane
  else if (my_lane_index == 1)
  {
    bool left_lane_open = !is_lane_closed(0, car_state, sensor_data);
    bool right_lane_open = !is_lane_closed(2, car_state, sensor_data);
    if ((lanes_distances[0] > lanes_distances[2]) && left_lane_open){
      if (
        // move to side lane if it has big probability to pass
        // or its car have bigger speed
        (
          lanes_distances[0] > 1.4 * (safety_front_car_dist + safety_change_lane_gap) &&
          left_lane_open
        ) || (
          lanes_distances[0] > lanes_distances[1] &&
          lanes_speed[0] > lanes_speed[1] &&
          left_lane_open
        )
      )
      {
        return -1;
      } else {
        return 0;
      }
    } else {
      if (
        // move to side lane if it has big probability to pass
        // or its car have bigger speed
        (
          lanes_distances[2] > (safety_front_car_dist + safety_change_lane_gap) &&
          right_lane_open
        ) || (
          lanes_distances[2] > lanes_distances[1] &&
          lanes_speed[2] > lanes_speed[1] &&
          right_lane_open
        )
      )
      {
        return 1;
      } else {
        return 0;
      }
    }
  }
  //right lane
  else if (my_lane_index == 2)
  {
    bool lane_open = !is_lane_closed(1, car_state, sensor_data);
    // center lane is preffered for us
    if (
      lanes_time_to_collision[1] > safety_time_to_collision &&
      lanes_distances[1] > lanes_distances[2] &&
      lane_open
    ){
      return -1;
    } else {
      return 0;
    }
  }

  return 0;
}

/**
 * @brief is destination lane reached
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @return
 */
bool BehaviorLayer::is_dst_lane_reached (const std::vector<double>& car_state)
{
  double car_d = car_state[3];
  double dst_lane_center_d = dst_lane_index * 4 + 2.2;
  return fabs(dst_lane_center_d - car_d) < 0.5;
}

/**
 * @brief Central desision making method.
 *   Desides wether it is needed to change current state
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param sensor_data - information about other cars on the road
 * @return
 */
bool BehaviorLayer::update_current_state (
  const std::vector<double>& car_state,
  const std::vector<std::vector<double>>& sensor_data
)
{
  double car_d = car_state[3];

  // need to get lanes_values before decision making
  // lanes values are time to collision for current lane
  std::vector<double> lanes_time_to_collision;
  std::vector<double> lanes_distances;
  std::vector<double> lanes_speed;
  get_lanes_values (
    lanes_time_to_collision,
    lanes_distances,
    lanes_speed,
    car_state,
    sensor_data
  );

  CarState prev_state = current_state;

  if (current_state == CarState::START)
  {
    current_state = CarState::MOVE_FORWARD;
  }
  else if (current_state == CarState::MOVE_FORWARD)
  {
    int my_lane_index = get_lane_index (car_d);

    if (lanes_distances [my_lane_index] < safety_front_car_dist)
    {
      //we need to decide wether we need to:
      //1. change lane
      //2. follow next car in the lane
      int lane_index_change = try_change_lane (
        car_state,
        lanes_time_to_collision,
        lanes_distances,
        lanes_speed,
        sensor_data
      );
      switch(lane_index_change)
      {
        case -1:
          current_state = CarState::CHANGING_LEFT;
        break;
        case 0:
          current_state = CarState::FOLLOW;
        break;
        case 1:
          current_state = CarState::CHANGING_RIGHT;
        break;
      }
      dst_lane_index = my_lane_index + lane_index_change;
    }
    else if (
      my_lane_index != 1 &&
      lanes_distances[my_lane_index] <= lanes_distances[1]
    )
    {
      bool center_lane_open = !is_lane_closed(1, car_state, sensor_data);
      if (center_lane_open) {
        current_state = my_lane_index > 1 ? CarState::CHANGING_LEFT : CarState::CHANGING_RIGHT;
        dst_lane_index = 1;
      }
    }
  }

  else if (
    current_state == CarState::CHANGING_LEFT ||
    current_state == CarState::CHANGING_RIGHT
  )
  {
    if (is_dst_lane_reached(car_state)) {
      current_state = CarState::MOVE_FORWARD;
      update_current_state (car_state, sensor_data);
    };
  }

  else if (current_state == CarState::FOLLOW)
  {
    int my_lane_index = get_lane_index (car_d);
    // if we decreased veclocity because of road curvature and
    // left followed car we need to FORWARD
    if (lanes_distances [my_lane_index] < 0.8 * safety_front_car_dist) {
      current_state = CarState::FOLLOW_SLOWING;
    } else if (lanes_distances [my_lane_index] > 2.0 * safety_front_car_dist){
      current_state = CarState::MOVE_FORWARD;
    } else {

      int lane_index_change = try_change_lane (
        car_state,
        lanes_time_to_collision,
        lanes_distances,
        lanes_speed,
        sensor_data
      );
      switch(lane_index_change)
      {
       case -1:
         current_state = CarState::CHANGING_LEFT;
       break;
       case 1:
         current_state = CarState::CHANGING_RIGHT;
       break;
      }
      dst_lane_index = my_lane_index + lane_index_change;

    }
  }

  else if (current_state == CarState::FOLLOW_SLOWING)
  {
    int my_lane_index = get_lane_index (car_d);
    if (lanes_distances [my_lane_index] > safety_front_car_dist) {
      current_state = CarState::FOLLOW;
    }
  }

  return prev_state != current_state;
}

/**
 * @brief process input from simulator
 * @param next_x_vals
 * @param next_y_vals
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param previous_path_x
 * @param previous_path_y
 * @param sensor_data - information about other cars on the road
 */
void BehaviorLayer::process_step (
  std::vector<double>& next_x_vals,
  std::vector<double>& next_y_vals,
  const std::vector<double>& car_state,
  const std::vector<double>& previous_path_x,
  const std::vector<double>& previous_path_y,
  const std::vector<std::vector<double>>& sensor_data
)
{
  bool state_changed = update_current_state (car_state, sensor_data);

  double car_s = car_state[2];
  double car_d = car_state[3];

  if (state_changed || previous_path_x.size() < maneuver_recalc_steps_count)
  {
    std::vector<double> next_s_vals, next_d_vals;

    std::vector<double> splined_waypoints_s;
    std::vector<double> splined_waypoints_x;
    std::vector<double> splined_waypoints_y;
    getSplinedMapPoints(
      car_s,
      splined_waypoints_s,
      splined_waypoints_x,
      splined_waypoints_y,
      map_waypoints_s,
      map_waypoints_x,
      map_waypoints_y
    );
    double curv = getMaxCurvatureOfRoad(car_s, car_d, 50, splined_waypoints_s, splined_waypoints_x, splined_waypoints_y);
    double curv_max_speed = std::sqrt(2.0 / curv);
    double allowed_forward_speed = std::min(forward_speed, curv_max_speed);

    if (current_state == CarState::MOVE_FORWARD)
    {
      calc_move_forward(next_s_vals, next_d_vals, car_state, allowed_forward_speed);
    }
    else if (
      current_state == CarState::CHANGING_LEFT ||
      current_state == CarState::CHANGING_RIGHT
    )
    {
      calc_change_lane(
        next_s_vals,
        next_d_vals,
        car_state,
        dst_lane_index
      );
    }
    else if (current_state == CarState::FOLLOW)
    {
      calc_move_forward(
        next_s_vals,
        next_d_vals,
        car_state,
        std::min(allowed_forward_speed, get_front_car_speed(
          car_state,
          sensor_data
        ))
      );
    }
    else if (current_state == CarState::FOLLOW_SLOWING)
    {

      std::cout << " state: " << int(current_state) << " fcs " << get_front_car_speed(car_state, sensor_data) << " : " << std::min(allowed_forward_speed, 0.6 * get_front_car_speed(
                                                                                                                                     car_state,
                                                                                                                                     sensor_data
                                                                                                                                   ))<<std::endl;

      calc_move_forward(
        next_s_vals,
        next_d_vals,
        car_state,
        std::min(allowed_forward_speed, 0.8 * get_front_car_speed(
          car_state,
          sensor_data
        ))
      );
    }


    std::vector<double> new_xs, new_ys;

    for (int i=0; i<next_s_vals.size(); i++) {
      std::vector<double> xy = getXY(next_s_vals[i], next_d_vals[i], splined_waypoints_s, splined_waypoints_x, splined_waypoints_y);
      new_xs.push_back(xy[0]);
      new_ys.push_back(xy[1]);
    }
    for(int i=0; i<5; i++)
    {
      TrajectorySmoother::smooth_trajectory(new_xs);
      TrajectorySmoother::smooth_trajectory(new_ys);
    }

    // keep 10 steps from old trajectory and transite into new in 100 steps
    TrajectorySmoother::merge_trajectoies(next_x_vals, previous_path_x, new_xs, 10, 100);
    TrajectorySmoother::merge_trajectoies(next_y_vals, previous_path_y, new_ys, 10, 100);
  }
  else
  {
    int passed_steps = maneuver_planner.get_steps_left() - previous_path_x.size();
    maneuver_planner.update_maneuver(
      passed_steps
    );
    next_x_vals.resize(previous_path_x.size());
    next_y_vals.resize(previous_path_y.size());
    copy(previous_path_x.begin(), previous_path_x.end(), next_x_vals.begin());
    copy(previous_path_y.begin(), previous_path_y.end(), next_y_vals.begin());
  }

}

/**
 * @brief Calculate move forward in lane trajectory
 * @param next_s_vals - output trajectory s values
 * @param next_d_vals - output trajectory d values
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param dst_speed
 */
void BehaviorLayer::calc_move_forward (
  std::vector<double>& next_s_vals,
  std::vector<double>& next_d_vals,
  const std::vector<double>& car_state,
  double dst_speed
)
{
  double car_s = car_state[2];
  double car_d = car_state[3];
  int my_lane_index = get_lane_index (car_state[3]);
  double my_lane_position = get_lane_d (my_lane_index);

  maneuver_planner.calc_acceleration(
    next_s_vals,
    next_d_vals,
    maneuver_min_steps_count,
    car_s, // start_s
    dst_speed, // end_s_dot
    car_d, // start_d
    my_lane_position // end_d
  );
}

/**
 * @brief Calculate change lane trajectory
 * @param next_s_vals - output trajectory s values
 * @param next_d_vals - output trajectory d values
 * @param car_state - vector [x, y, s, d, yaw, v]
 * @param new_lane_index
 */
void BehaviorLayer::calc_change_lane (
  std::vector<double>& next_s_vals,
  std::vector<double>& next_d_vals,
  const std::vector<double>& car_state,
  int new_lane_index
)
{
  double car_s = car_state[2];
  double car_d = car_state[3];
  double new_lane_position = get_lane_d (new_lane_index);

  maneuver_planner.calc_change_lane(
    next_s_vals,
    next_d_vals,
    maneuver_min_steps_count,
    car_s, // start_s
    car_d, // start_d
    new_lane_position // end_d
  );
}
