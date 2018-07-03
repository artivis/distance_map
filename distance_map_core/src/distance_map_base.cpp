#include "distance_map_core/distance_map_base.h"

#include <ros/ros.h>

namespace distmap {

bool DistanceMapBase::process(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  throw std::runtime_error("Not implemented !");
  return true;
}

/*
double DistanceMapBase::cost(const double x, const double y)
{
  return 50;
}

double DistanceMapBase::cost(const double x, const double y, const double)
{
  ROS_WARN("Cost given yaw is not implemented. "
           "Falling back to cost(x,y).");

  return cost(x,y);
}
*/

}
