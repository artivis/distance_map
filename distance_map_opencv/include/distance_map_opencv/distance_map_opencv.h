#ifndef _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_
#define _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_

#include <distance_map_core/distance_map_converter_base.h>

#include <opencv2/opencv.hpp>

namespace distmap {

class DistanceMapOpencv : public DistanceMapConverterBase
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

  int distance_type_ = cv::DIST_L2,
      mask_size_     = cv::DIST_MASK_PRECISE;

  cv::Mat image_,
          binary_image_,
          distance_field_obstacle_image_;

  bool configureImpl() override;
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_OPENCV_DISTANCE_MAP_OPENCV_H_ */
