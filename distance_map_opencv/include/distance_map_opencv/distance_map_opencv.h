/*
 * Copyright 2019 PAL Robotics SL.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Sai Kishor Kothakota,
 *          Jeremie Deray
 */

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
                                     distmap::DistanceMap &map);
  static void matToDistanceFieldGrid(const cv::Mat& cv_map,
                                     const nav_msgs::MapMetaData& map_metadata,
                                     distmap::DistanceMap &map);

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
