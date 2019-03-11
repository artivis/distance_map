#include "distance_map_deadreck/distance_map_deadreck.h"
#include "distance_map_msgs/DistanceFieldGrid.h"

#include <ros/ros.h>
#include <costmap_2d/cost_values.h>

#define SDT_DEAD_RECKONING_IMPLEMENTATION
#include "distance_map_deadreck/sdt_dead_reckoning.h"

namespace distmap {

/*
bool DistanceMapDeadReck::configureImpl()
{
  ros::NodeHandle nh("~/DistanceMapDeadReck");

  return true;
}
*/

bool DistanceMapDeadReck::processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  assert(field_obstacles_ && "field_obstacles_ is nullptr !");
  assert(field_unknowns_  && "field_unknowns_ is nullptr !");

  if (occ_grid == nullptr)
  {
    ROS_WARN("Received a nav_msgs::OccupancyGridConstPtr nullptr !");
    return false;
  }

  if (previous_dim_.width  != occ_grid->info.width  ||
      previous_dim_.height != occ_grid->info.height   )
  {
    img_ptr_.reset( new unsigned char[occ_grid->info.width*occ_grid->info.height] );
    distance_field_ptr_.reset( new float[occ_grid->info.width*occ_grid->info.height] );
    previous_dim_.width  = occ_grid->info.width;
    previous_dim_.height = occ_grid->info.height;
  }

  std::transform(occ_grid->data.data(),
                 occ_grid->data.data() + (occ_grid->info.width*occ_grid->info.height),
                 img_ptr_.get(),
                 [](nav_msgs::OccupancyGrid::_data_type::value_type d) -> unsigned char
                 {
                    return d==0? 0 : d==100? 255 : 127;
                 });

  const unsigned char threshold = unknow_is_obstacle_? 126 : 128;

  /// @todo pre-allocate inner buffers
  sdt_dead_reckoning(occ_grid->info.width, occ_grid->info.height,
                     threshold,
                     img_ptr_.get(), distance_field_ptr_.get());

  unsigned int i, t = 0;
  for (unsigned int row = 0; row < occ_grid->info.height; ++row) {
    for (unsigned int col = 0; col < occ_grid->info.width; ++col) {
      i = col + (occ_grid->info.height - row - 1) * occ_grid->info.width;
      field_obstacles_->data()[col + row * occ_grid->info.width] = distance_field_ptr_.get()[i];
    }
  }

  return true;
}

bool DistanceMapDeadReck::processImpl(const costmap_2d::Costmap2D* cost_map)
{
  assert(field_obstacles_ && "field_obstacles_ is nullptr !");
  assert(field_unknowns_  && "field_unknowns_ is nullptr !");

  if (cost_map == nullptr)
  {
    ROS_WARN("Received a costmap_2d::Costmap2D* nullptr !");
    return false;
  }

  return true;
}

} /* namespace distmap */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(distmap::DistanceMapDeadReck, distmap::DistanceMapBase);
