#include "distance_map_core/distance_map_base.h"

#include <ros/ros.h>

namespace distmap {

bool DistanceMapBase::configure()
{
//  field_obstacles_ = boost::make_shared<DistanceFieldGrid>();
//  field_unknowns_  = boost::make_shared<DistanceFieldGrid>();

  return configureImpl();
}

bool DistanceMapBase::process(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  if (occ_grid == nullptr)
  {
    ROS_ERROR("Input nav_msgs::OccupancyGridConstPtr is nullptr !");
    return false;
  }

  if (field_obstacles_ == nullptr)
  {
    field_obstacles_ = std::make_shared<DistanceFieldGrid>(
                         DistanceFieldGrid::Dimension(occ_grid->info.width,
                                                      occ_grid->info.height),
                         occ_grid->info.resolution,
                         DistanceFieldGrid::Origin(occ_grid->info.origin.position.x,
                                                   occ_grid->info.origin.position.y,
                                                   0));
  }

  if (field_unknowns_ == nullptr)
  {
    field_obstacles_ = std::make_shared<DistanceFieldGrid>(
                         DistanceFieldGrid::Dimension(occ_grid->info.width,
                                                      occ_grid->info.height),
                         occ_grid->info.resolution,
                         DistanceFieldGrid::Origin(occ_grid->info.origin.position.x,
                                                   occ_grid->info.origin.position.y,
                                                   0));
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

} /* namespace distmap */
