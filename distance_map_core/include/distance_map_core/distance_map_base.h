#ifndef _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_
#define _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>

#include <distance_map_msgs/DistanceFieldGrid.h>

#include "distance_map_core/distance_field_grid.h"

namespace distmap {

class DistanceMapBase
{
public:

  DistanceMapBase() = default;
  virtual ~DistanceMapBase() = default;

  virtual bool process(const nav_msgs::OccupancyGridConstPtr occ_grid);
  virtual bool process(const costmap_2d::Costmap2D* cost_map);
  //virtual bool convert(const nav_msgs::OccupancyGridConstPtr occ_grid);

  //double cost(const double x, const double y);
  //double cost(const double x, const double y, const double yaw);

  void setType(const std::string& type);

  bool configure();

  DistanceFieldGridPtr getDistanceFieldObstacle();

  DistanceFieldGridPtr getDistanceFieldUnknown();

protected:

  std::string type_;

  DistanceFieldGridPtr field_obstacles_,
                       field_unknowns_;

  virtual bool configureImpl();

  virtual void preProcess(const nav_msgs::OccupancyGridConstPtr occ_grid);
  virtual void preProcess(const costmap_2d::Costmap2D* cost_map);

  virtual bool processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid) = 0;
  virtual bool processImpl(const costmap_2d::Costmap2D* cost_map) = 0;

  virtual void postProcess();
};

using DistanceMapPtr = boost::shared_ptr<DistanceMapBase>;

inline void DistanceMapBase::setType(const std::string& type) { type_ = type; }

inline bool DistanceMapBase::configureImpl()
{
  return true;
}

inline void DistanceMapBase::preProcess(const nav_msgs::OccupancyGridConstPtr)
{
  //
}

inline void DistanceMapBase::preProcess(const costmap_2d::Costmap2D*)
{
  //
}

bool DistanceMapBase::processImpl(const nav_msgs::OccupancyGridConstPtr)
{
  throw std::runtime_error("DistanceMapBase::processImpl is not implemented "
                           "for input type nav_msgs::OccupancyGridConstPtr !");
  return false;
}

bool DistanceMapBase::processImpl(const costmap_2d::Costmap2D*)
{
  throw std::runtime_error("DistanceMapBase::processImpl is not implemented "
                           "for input type costmap_2d::Costmap2D !");
  return false;
}

inline void DistanceMapBase::postProcess()
{
  //
}

inline DistanceFieldGridPtr
DistanceMapBase::getDistanceFieldObstacle()
{
  return field_obstacles_;
}

inline DistanceFieldGridPtr
DistanceMapBase::getDistanceFieldUnknown()
{
  return field_unknowns_;
}

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_ */
