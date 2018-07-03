#ifndef _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_
#define _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_

#include <nav_msgs/OccupancyGrid.h>

#include "distance_map_msgs/DistanceFieldGrid.h"

namespace distmap {

class DistanceMapBase
{
public:

  DistanceMapBase() = default;
  virtual ~DistanceMapBase() = default;

  virtual bool process(const nav_msgs::OccupancyGridConstPtr occ_grid);
  //virtual bool convert(const nav_msgs::OccupancyGridConstPtr occ_grid);

  //double cost(const double x, const double y);
  //double cost(const double x, const double y, const double yaw);

  void setType(const std::string& type) { type_ = type; }

  inline virtual bool configure() { return true; };

protected:

  std::string type_;

  distance_map_msgs::DistanceFieldGrid field_obstacles_,
                                       field_unknowns_;

  virtual void preProcess() {};
  virtual void postProcess() {};
};

using DistanceMapPtr = boost::shared_ptr<DistanceMapBase>;

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_DISTANCE_MAP_BASE_H_ */
