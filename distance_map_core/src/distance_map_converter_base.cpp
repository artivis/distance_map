#include "distance_map_core/distance_map_converter_base.h"

#include <ros/ros.h>
#include <tf2/utils.h>

namespace distmap {

bool DistanceMapConverterBase::configure()
{
  return configureImpl();
}

bool DistanceMapConverterBase::process(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  if (occ_grid == nullptr)
  {
    ROS_ERROR("Input nav_msgs::OccupancyGridConstPtr is nullptr !");
    return false;
  }

  if (field_obstacles_ == nullptr or
      field_obstacles_->getDimension().width  != occ_grid->info.width or
      field_obstacles_->getDimension().height != occ_grid->info.height)
  {
    field_obstacles_ = std::make_shared<DistanceMap>(
                         DistanceMap::Dimension(occ_grid->info.width,
                                                      occ_grid->info.height),
                         occ_grid->info.resolution,
                         DistanceMap::Origin(occ_grid->info.origin.position.x,
                                                   occ_grid->info.origin.position.y,
                                                   tf2::getYaw(occ_grid->info.origin.orientation)));
  }
  else if (field_obstacles_->getOrigin().x != occ_grid->info.origin.position.x or
           field_obstacles_->getOrigin().y != occ_grid->info.origin.position.y)
  {
    field_obstacles_->setOrigin(
          DistanceMap::Origin(occ_grid->info.origin.position.x,
                                    occ_grid->info.origin.position.y,
                                    tf2::getYaw(occ_grid->info.origin.orientation)));
  }

  if (field_unknowns_ == nullptr or
      field_unknowns_->getDimension().width  != occ_grid->info.width or
      field_unknowns_->getDimension().height != occ_grid->info.height)
  {
    field_unknowns_ = std::make_shared<DistanceMap>(
                         DistanceMap::Dimension(occ_grid->info.width,
                                                      occ_grid->info.height),
                         occ_grid->info.resolution,
                         DistanceMap::Origin(occ_grid->info.origin.position.x,
                                                   occ_grid->info.origin.position.y,
                                                   tf2::getYaw(occ_grid->info.origin.orientation)));
  }
  else if (field_unknowns_->getOrigin().x != occ_grid->info.origin.position.x or
           field_unknowns_->getOrigin().y != occ_grid->info.origin.position.y)
  {
    field_unknowns_->setOrigin(
          DistanceMap::Origin(occ_grid->info.origin.position.x,
                                    occ_grid->info.origin.position.y,
                                    tf2::getYaw(occ_grid->info.origin.orientation)));
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

bool DistanceMapConverterBase::process(const costmap_2d::Costmap2D* cost_map)
{
  if (cost_map == nullptr)
  {
    ROS_ERROR("Input costmap_2d::Costmap2D* is nullptr !");
    return false;
  }

  const double resolution = cost_map->getResolution();

  /// @note it would make sense to account for the resolution
  /// but it seems that the local costmap does not, therefor a
  /// slight offset appears in visualization which could be confusing.
//  const double resolution_offset = resolution / 2;
  const double resolution_offset = 0;

  if (field_obstacles_ == nullptr or
      field_obstacles_->getDimension().width  != cost_map->getSizeInCellsX() or
      field_obstacles_->getDimension().height != cost_map->getSizeInCellsY())
  {
    field_obstacles_ = std::make_shared<DistanceMap>(
                         DistanceMap::Dimension(cost_map->getSizeInCellsX(),
                                                      cost_map->getSizeInCellsY()),
                         resolution,
                         DistanceMap::Origin(cost_map->getOriginX() - resolution_offset,
                                                   cost_map->getOriginY() - resolution_offset,
                                                   0));
  }
  if (field_obstacles_->getOrigin().x != (cost_map->getOriginX() - resolution_offset) or
      field_obstacles_->getOrigin().y != (cost_map->getOriginY() - resolution_offset))
  {
    field_obstacles_->setOrigin(
          DistanceMap::Origin(cost_map->getOriginX() - resolution_offset,
                                    cost_map->getOriginY() - resolution_offset,
                                    0));
  }

  /// @todo handle resizing
  if (field_unknowns_ == nullptr or
      field_unknowns_->getDimension().width  != cost_map->getSizeInCellsX() or
      field_unknowns_->getDimension().height != cost_map->getSizeInCellsY())
  {
    double resolution = cost_map->getResolution();
    field_unknowns_ = std::make_shared<DistanceMap>(
                         DistanceMap::Dimension(cost_map->getSizeInCellsX(),
                                                      cost_map->getSizeInCellsY()),
                         resolution,
                         DistanceMap::Origin(cost_map->getOriginX() - resolution_offset,
                                                   cost_map->getOriginY() - resolution_offset,
                                                   0));
  }
  if (field_unknowns_->getOrigin().x != (cost_map->getOriginX() - resolution_offset) or
      field_unknowns_->getOrigin().y != (cost_map->getOriginY() - resolution_offset))
  {
    field_unknowns_->setOrigin(
          DistanceMap::Origin(cost_map->getOriginX() - resolution_offset,
                                    cost_map->getOriginY() - resolution_offset,
                                    0));
  }

  preProcess(cost_map);

  bool processed = processImpl(cost_map);

  if (!processed)
  {
    ROS_WARN("Could not process Costmap2D !");
  }

  postProcess();

  return processed;
}

} /* namespace distmap */
