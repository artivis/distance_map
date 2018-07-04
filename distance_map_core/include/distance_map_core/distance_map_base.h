#ifndef _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_
#define _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_

#include <nav_msgs/OccupancyGrid.h>

#include "distance_map_msgs/DistanceFieldGrid.h"

namespace distmap {

class DistanceMapBase
{
public:

  using DistanceFieldGrid    = distance_map_msgs::DistanceFieldGrid;
  using DistanceFieldGridPtr = distance_map_msgs::DistanceFieldGridPtr;

  DistanceMapBase() = default;
  virtual ~DistanceMapBase() = default;

  virtual bool process(const nav_msgs::OccupancyGridConstPtr occ_grid);
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

  virtual bool processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid) = 0;

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

inline void DistanceMapBase::postProcess()
{
  //
}

inline DistanceMapBase::DistanceFieldGridPtr
DistanceMapBase::getDistanceFieldObstacle()
{
  return field_obstacles_;
}

inline DistanceMapBase::DistanceFieldGridPtr
DistanceMapBase::getDistanceFieldUnknown()
{
  return field_unknowns_;
}

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_ */
