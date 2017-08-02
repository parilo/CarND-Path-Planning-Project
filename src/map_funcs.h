#ifndef MAP_FUNCS_H
#define MAP_FUNCS_H

#include <vector>

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(
  double x,
  double y,
  double theta,
  std::vector<double> maps_x,
  std::vector<double> maps_y
);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(
  double s,
  double d,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
);

std::vector<double> getXYSplined(
  double s,
  double d,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
);

std::vector<double> getXYPolyfit(
  double s,
  double d,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y
);

#endif /* MAP_FUNCS_H */
