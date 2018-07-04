#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "distance_map_opencv/distance_map_opencv.h"

#include "distance_map_msgs/DistanceFieldGrid.h"

namespace distmap  {

enum MapMode {TRINARY, SCALE, RAW};

struct Metadata : nav_msgs::OccupancyGrid::_info_type
{
  Metadata()  = default;
  ~Metadata() = default;

  int negate;
  double occupied_threshold;
  double free_threshold;
  MapMode mode;
  std::string image_file;
};

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

void matToDistanceFieldGridMsg(const cv::Mat& cv_map, const nav_msgs::MapMetaData map_metadata, distance_map_msgs::DistanceFieldGrid &map)
{
  map.info = map_metadata;
  map.data.resize(map.info.height * map.info.width);

  unsigned int i;

  double min, max;
  cv::minMaxLoc(cv_map, &min, &max);
  ROS_DEBUG("The max in the cv mat image is %4.2f", max);

  const cv::Mat& redressed_cv_map = cv_map;
  max = 0.0;

  for (unsigned int rows = 0; rows < map.info.height; ++rows) {
    for (unsigned int cols = 0; cols < map.info.width; ++cols) {

      i = cols + (map.info.height - rows - 1) * map.info.width;
      map.data[i] = redressed_cv_map.at<float>(rows, cols);
      if(max < redressed_cv_map.at<float>(rows, cols))
          max = redressed_cv_map.at<float>(rows, cols);

//          if (redressed_cv_map.at<uchar>(rows, cols) == 254) { //occ [0,0.1)
//            map.data[i] = 0;
//          } else if (redressed_cv_map.at<uchar>(rows, cols) == 0) { //occ (0.65,1]
//            map.data[i] = +100;
//          } else { //occ [0.1,0.65]
//            map.data[i] = -1;
//          }
    }
  }
  ROS_DEBUG("The max in the distance field is %4.2f", max);
}

bool DistanceMapOpencv::process(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  if (occ_grid == nullptr)
  {
    ROS_WARN("Received a nullptr !");
    return false;
  }

  // OccupancyGrid to cv::Mat 8-bit single channel
  occupancyGridToMat(*occ_grid).convertTo(image_, CV_8UC1);
  map_metadata_ = occ_grid->info;

  // conversion into the binary image
  cv::threshold(image_, binary_image_, 55, 255, cv::THRESH_BINARY);

  // computation of the distance transform on the binary image
  cv::distanceTransform(binary_image_, distance_field_obstacle_image_, CV_DIST_L2, 3);

  // Vizualization of the distance field image of obstacles
  /*
  cv::Mat distance_image_norm_;
  cv::normalize(distance_field_obstacle_image_, distance_image_norm_, 255, 0);
  cv::imshow("Display window", distance_image_norm_);
  cv::waitKey(0.2);
  */

  // convert opencv the distance_map_msgs
  matToDistanceFieldGridMsg(distance_field_obstacle_image_, map_metadata_, field_obstacles_);

  return true;
}

} /* namespace distmap */

PLUGINLIB_EXPORT_CLASS(distmap::DistanceMapOpencv, distmap::DistanceMapBase);
