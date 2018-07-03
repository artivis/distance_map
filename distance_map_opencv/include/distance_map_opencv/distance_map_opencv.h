#ifndef _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_
#define _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_

#include <distance_map_core/distance_map_base.h>

#include <opencv2/core/mat.hpp>

namespace distmap {

class DistanceMapOpencv : public DistanceMapBase
{
public:

  DistanceMapOpencv() = default;
  virtual ~DistanceMapOpencv() = default;

  virtual bool process(const nav_msgs::OccupancyGridConstPtr occ_grid);
  //virtual bool convert(const nav_msgs::OccupancyGridConstPtr occ_grid);

protected:

  nav_msgs::MapMetaData map_metadata_;

  cv::Mat image_,
          binary_image_,
          binary_image_unknown_,
          distance_field_obstacle_image_,
          distance_field_unknown_image_;

  //virtual void preProcess() override {};
  //virtual void postProcess() override {};

  //inline virtual bool configure() override { return true; };
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_ */
