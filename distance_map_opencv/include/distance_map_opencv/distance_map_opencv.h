#ifndef _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_
#define _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_

#include <distance_map_core/distance_map_base.h>

#include <opencv2/core/mat.hpp>

namespace distmap {

class DistanceMapOpencv : public DistanceMapBase
{
public:

  DistanceMapOpencv()  = default;
  ~DistanceMapOpencv() = default;

  bool processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid) override;
  bool processImpl(const costmap_2d::Costmap2D* cost_map) override;

  static cv::Mat occupancyGridToMat(const nav_msgs::OccupancyGrid& map);
  static cv::Mat costMapToMat(const costmap_2d::Costmap2D& costmap);
  static void matToDistanceFieldGrid(const cv::Mat& cv_map,
                                     const double resolution,
                                     distmap::DistanceFieldGrid &map);
  static void matToDistanceFieldGrid(const cv::Mat& cv_map,
                                     const nav_msgs::MapMetaData& map_metadata,
                                     distmap::DistanceFieldGrid &map);

protected:

  cv::Mat image_,
          binary_image_,
          binary_image_unknown_,
          distance_field_obstacle_image_,
          distance_field_unknown_image_;

  //inline virtual bool configure() override { return true; };
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_ */
