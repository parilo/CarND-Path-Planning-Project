#ifndef MAP_FUNCS_H
#define MAP_FUNCS_H

#include <vector>

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

/** Transform from Cartesian x,y coordinates to Frenet s,d coordinates
 * @brief getFrenet
 * @param x
 * @param y
 * @param theta
 * @param maps_x
 * @param maps_y
 * @return vector [s, d]
 */
std::vector<double> getFrenet(
  double x,
  double y,
  double theta,
  std::vector<double> maps_x,
  std::vector<double> maps_y
);

/**
 * @brief Transform from Frenet s,d coordinates to Cartesian x,y
 * @param s
 * @param d
 * @param maps_s
 * @param maps_x
 * @param maps_y
 * @return vector [x, y]
 */
std::vector<double> getXY(
  double s,
  double d,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
);

/**
 * @brief Get maximum curvature of specified length road segment
 * @param s - current s
 * @param d - current d
 * @param length - segment length, begins from s
 * @param maps_s - maps s keypoints
 * @param maps_x - maps x keypoints
 * @param maps_y - maps y keypoints
 * @return
 */
double getMaxCurvatureOfRoad(
  double s,
  double d,
  double length,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
);

/**
 * @brief Interpolate with splines sparsed maps keyupoints
 * @param s - current s
 * @param splined_maps_s - output interpolated values
 * @param splined_maps_x - output interpolated values
 * @param splined_maps_y - output interpolated values
 * @param maps_s - maps keypoints
 * @param maps_x - maps keypoints
 * @param maps_y - maps keypoints
 */
void getSplinedMapPoints(
  double s,
  std::vector<double>& splined_maps_s,
  std::vector<double>& splined_maps_x,
  std::vector<double>& splined_maps_y,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
);

#endif /* MAP_FUNCS_H */
