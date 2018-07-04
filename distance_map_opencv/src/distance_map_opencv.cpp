#include "distance_map_opencv/distance_map_opencv.h"
#include "distance_map_msgs/DistanceFieldGrid.h"

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

namespace distmap {

cv::Mat occupancyGridToMat(const nav_msgs::OccupancyGrid& map)
{
  cv::Mat cv_map(map.info.height, map.info.width, CV_8UC1);
  unsigned int i;
  // Make a double loop over indexes and assign values
  for (unsigned int rows = 0; rows < map.info.height; ++rows) {
    for (unsigned int cols = 0; cols < map.info.width; ++cols) {
      i = cols + (map.info.height - rows - 1) * map.info.width;
      if (map.data[i] == 0) { //occ [0,0.1)
        cv_map.at<uchar>(rows, cols) = 254;
      } else if (map.data[i] == +100) { //occ (0.65,1]
        cv_map.at<uchar>(rows, cols) = 0;
      } else { //occ [0.1,0.65]
        cv_map.at<uchar>(rows, cols) = 205;
      }
    }
  }

  /// @todo cv::Mat origin is top-left corner
  /// thus this image seems rotated by 90deg CC
  /// should I redress that ??
  //  cv::rotate(cv_map, cv_map, cv::ROTATE_90_CLOCKWISE);

  return cv_map;
}

void matToDistanceFieldGridMsg(const cv::Mat& cv_map,
                               const nav_msgs::MapMetaData map_metadata,
                               distance_map_msgs::DistanceFieldGrid &map)
{
  map.info = map_metadata;
  map.data.resize(map.info.height * map.info.width);

  unsigned int i;
  for (unsigned int rows = 0; rows < map.info.height; ++rows) {
    for (unsigned int cols = 0; cols < map.info.width; ++cols) {
      i = cols + rows * map.info.width;
      map.data[i] = cv_map.at<float>(rows, cols) * map.info.resolution;
    }
  }
}

bool DistanceMapOpencv::processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  assert(field_obstacles_ && "field_obstacles_ is nullptr !");
  assert(field_unknowns_  && "field_unknowns_ is nullptr !");

  if (occ_grid == nullptr)
  {
    ROS_WARN("Received a nullptr !");
    return false;
  }

  // OccupancyGrid to cv::Mat 8-bit single channel
  image_ = occupancyGridToMat(*occ_grid);

  // conversion into the binary image
  cv::threshold(image_, binary_image_, 55, 255, cv::THRESH_BINARY);

  // computation of the distance transform on the binary image
  cv::distanceTransform(binary_image_, distance_field_obstacle_image_, CV_DIST_L2, 3);

  // pixel-destance to meters
  distance_field_obstacle_image_ *= occ_grid->info.resolution;

  /*
  // Vizualization of the obstacles distance field grid
  cv::Mat distance_image_norm_;
  cv::normalize(distance_field_obstacle_image_, distance_image_norm_, 1, 0, cv::NORM_MINMAX);
  cv::imshow("original", binary_image_);
  cv::imshow("distances", distance_image_norm_);

  const double sec  = 1000;
  const double wait = 30 * sec;

  cv::waitKey(wait);
  cv::destroyAllWindows();
  */

  // convert opencv the distance_map_msgs
  matToDistanceFieldGridMsg(distance_field_obstacle_image_,
                            occ_grid->info, *field_obstacles_);

  field_obstacles_->header = occ_grid->header;

  return true;
}

} /* namespace distmap */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(distmap::DistanceMapOpencv, distmap::DistanceMapBase);
