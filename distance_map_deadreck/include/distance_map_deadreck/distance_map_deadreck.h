#ifndef _DISTANCE_MAP_DEADRECK_DISTANCE_MAP_DEADRECK_H_
#define _DISTANCE_MAP_DEADRECK_DISTANCE_MAP_DEADRECK_H_

#include <distance_map_core/distance_map_base.h>

namespace distmap {

class DistanceMapDeadReck : public DistanceMapBase
{
public:

  DistanceMapDeadReck()  = default;
  ~DistanceMapDeadReck() = default;

  bool processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid) override;
  bool processImpl(const costmap_2d::Costmap2D* cost_map) override;

protected:

  DistanceFieldGrid::Dimension previous_dim_ = DistanceFieldGrid::Dimension(1,1);
  std::unique_ptr<unsigned char[]> img_ptr_;
  std::unique_ptr<float[]> distance_field_ptr_;

  //bool configureImpl() override { return true; };
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_DEADRECK_DISTANCE_MAP_DEADRECK_H_ */
