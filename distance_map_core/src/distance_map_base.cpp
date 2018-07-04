#include "distance_map_core/distance_map_base.h"

#include <ros/ros.h>

namespace distmap {

bool DistanceMapBase::configure()
{
  field_obstacles_ = boost::make_shared<DistanceFieldGrid>();
  field_unknowns_  = boost::make_shared<DistanceFieldGrid>();

  return configureImpl();
}

bool DistanceMapBase::process(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  if (occ_grid == nullptr)
  {
    ROS_ERROR("Input nav_msgs::OccupancyGridConstPtr is nullptr !");
    return false;
  }

  preProcess(occ_grid);

  bool processed = processImpl(occ_grid);

  if (!processed)
  {
    ROS_WARN("Could not process occupancy_grid !");
  }

  postProcess();

  return processed;
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
