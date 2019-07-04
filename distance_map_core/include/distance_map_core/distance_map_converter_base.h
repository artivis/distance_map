#ifndef _DISTANCE_MAP_CORE_DISTANCE_MAP_CONVERTER_BASE_H_
#define _DISTANCE_MAP_CORE_DISTANCE_MAP_CONVERTER_BASE_H_

#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d.h>

#include <distance_map_msgs/DistanceMap.h>

#include "distance_map_core/distance_map.h"

namespace distmap {

class DistanceMapConverterBase
{
public:

  DistanceMapConverterBase() = default;
  virtual ~DistanceMapConverterBase() = default;

  virtual bool process(const nav_msgs::OccupancyGridConstPtr occ_grid);
  virtual bool process(const costmap_2d::Costmap2D* cost_map);

  void setType(const std::string& type);

  bool configure();

  DistanceFieldGridPtr getDistanceFieldObstacle();

  DistanceFieldGridPtr getDistanceFieldUnknown();

  void setUnknowObstacles(bool unknow_is_obstacle);
  bool getUnknowObstacles() const noexcept;

protected:

  bool unknow_is_obstacle_ = true;

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

using DistanceMapPtr = boost::shared_ptr<DistanceMapConverterBase>;

inline void DistanceMapConverterBase::setType(const std::string& type)
{
  type_ = type;
}

inline bool DistanceMapConverterBase::configureImpl()
{
  return true;
}

inline void DistanceMapConverterBase::preProcess(const nav_msgs::OccupancyGridConstPtr)
{
  //
}

inline void DistanceMapConverterBase::preProcess(const costmap_2d::Costmap2D*)
{
  //
}

inline bool DistanceMapConverterBase::processImpl(const nav_msgs::OccupancyGridConstPtr)
{
  throw std::runtime_error("DistanceMapConverterBase::processImpl is not implemented "
                           "for input type nav_msgs::OccupancyGridConstPtr !");
  return false;
}

inline bool DistanceMapConverterBase::processImpl(const costmap_2d::Costmap2D*)
{
  throw std::runtime_error("DistanceMapConverterBase::processImpl is not implemented "
                           "for input type costmap_2d::Costmap2D !");
  return false;
}

inline void DistanceMapConverterBase::postProcess()
{
  //
}

inline DistanceFieldGridPtr
DistanceMapConverterBase::getDistanceFieldObstacle()
{
  return field_obstacles_;
}

inline DistanceFieldGridPtr
DistanceMapConverterBase::getDistanceFieldUnknown()
{
  return field_unknowns_;
}

inline void DistanceMapConverterBase::setUnknowObstacles(bool unknow_is_obstacle)
{
  unknow_is_obstacle_ = unknow_is_obstacle;
}

inline bool DistanceMapConverterBase::getUnknowObstacles() const noexcept
{
  return unknow_is_obstacle_;
}

} // namespace distmap

#endif // _DISTANCE_MAP_CORE_DISTANCE_MAP_CONVERTER_BASE_H_
