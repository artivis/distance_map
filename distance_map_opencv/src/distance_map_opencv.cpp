#include "distance_map_opencv/distance_map_opencv.h"
#include "distance_map_msgs/DistanceFieldGrid.h"

#include <ros/ros.h>
#include <costmap_2d/cost_values.h>

#include <opencv2/opencv.hpp>

namespace distmap {

cv::Mat DistanceMapOpencv::occupancyGridToMat(const nav_msgs::OccupancyGrid& map)
{
  cv::Mat cv_map((int)map.info.height, (int)map.info.width, CV_8UC1);
  unsigned int i;

  for (unsigned int row = 0; row < map.info.height; ++row) {
    for (unsigned int col = 0; col < map.info.width; ++col) {
      i = row * map.info.width + col;
      if (map.data[i] == 0) { //occ [0,0.1)
        cv_map.at<uchar>(row, col) = 254;
      } else if (map.data[i] == +100) { //occ (0.65,1]
        cv_map.at<uchar>(row, col) = 0;
      } else { //occ [0.1,0.65]
        cv_map.at<uchar>(row, col) = 127; //205?
      }
    }
  }

  return cv_map;
}

cv::Mat DistanceMapOpencv::costMapToMat(const costmap_2d::Costmap2D& costmap)
{
  const unsigned int X = costmap.getSizeInCellsX(),
                     Y = costmap.getSizeInCellsY();
  cv::Mat cv_map(Y, X, CV_8UC1);
  unsigned int i;
  // Make a double loop over indexes and assign values
  unsigned char const* data = costmap.getCharMap();
  for (unsigned int row = 0; row < Y; ++row) {
    for (unsigned int col = 0; col < X; ++col) {
      i = col + (Y - row - 1) * X;
//      i = row * X + col;
      if (data[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE or
          data[i] == costmap_2d::LETHAL_OBSTACLE) { // Obstacle
        cv_map.at<uchar>(row, col) = 0;
      } else if (data[i] == costmap_2d::NO_INFORMATION) { // UNKNOWN
        cv_map.at<uchar>(row, col) = 127;
      } else { // FREE_SPACE
        cv_map.at<uchar>(row, col) = 254;
      }
    }
  }

  return cv_map;
}

void DistanceMapOpencv::matToDistanceFieldGrid(const cv::Mat& cv_map,
                                               const double resolution,
                                               distmap::DistanceFieldGrid &map)
{
  map.resize(cv_map.rows, cv_map.cols);
  map.setResolution(resolution);

  unsigned int i;
  for (int row = 0; row < cv_map.rows; ++row) {
    for (int col = 0; col < cv_map.cols; ++col) {
      i = (unsigned int)(row * cv_map.cols + col);
      map.data()[i] = static_cast<double>(cv_map.at<float>(row, col));
    }
  }
}

void DistanceMapOpencv::matToDistanceFieldGrid(const cv::Mat& cv_map,
                                               const nav_msgs::MapMetaData& map_metadata,
                                               distmap::DistanceFieldGrid &map)
{
  matToDistanceFieldGrid(cv_map,
                         map_metadata.resolution,
                         map);
}

bool DistanceMapOpencv::processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  assert(field_obstacles_ && "field_obstacles_ is nullptr !");
  assert(field_unknowns_  && "field_unknowns_ is nullptr !");

  if (occ_grid == nullptr)
  {
    ROS_WARN("Received a nav_msgs::OccupancyGridConstPtr nullptr !");
    return false;
  }

  // OccupancyGrid to cv::Mat 8-bit single channel
  image_ = occupancyGridToMat(*occ_grid);

  // conversion into the binary image
  double threshold = unknow_is_obstacle_? 128 : 126;
  cv::threshold(image_, binary_image_, threshold, 255, cv::THRESH_BINARY);

  // computation of the distance transform on the binary image
  cv::distanceTransform(binary_image_, distance_field_obstacle_image_, CV_DIST_L2, 3);

//  // Vizualization of the obstacles distance field grid
//  cv::Mat distance_image_norm_;
//  cv::normalize(distance_field_obstacle_image_, distance_image_norm_, 1, 0, cv::NORM_MINMAX);
//  cv::imshow("original", binary_image_);
//  cv::imshow("distances", distance_image_norm_);

//  static const double sec = 1000;
//  const double wait = 30 * sec;

//  cv::waitKey(wait);
//  cv::destroyAllWindows();

  // convert opencv to distance_map_msgs
  matToDistanceFieldGrid(distance_field_obstacle_image_,
                         occ_grid->info.resolution, *field_obstacles_);
  return true;
}

bool DistanceMapOpencv::processImpl(const costmap_2d::Costmap2D* cost_map)
{
  assert(field_obstacles_ && "field_obstacles_ is nullptr !");
  assert(field_unknowns_  && "field_unknowns_ is nullptr !");

  if (cost_map == nullptr)
  {
    ROS_WARN("Received a costmap_2d::Costmap2D* nullptr !");
    return false;
  }

  // OccupancyGrid to cv::Mat 8-bit single channel
  image_ = costMapToMat(*cost_map);

  // conversion into the binary image
  double threshold = unknow_is_obstacle_? 128 : 126;
  cv::threshold(image_, binary_image_, threshold, 255, cv::THRESH_BINARY);

  // computation of the distance transform on the binary image
  cv::distanceTransform(binary_image_, distance_field_obstacle_image_, CV_DIST_L2, 3);

  /*
  // Vizualization of the obstacles distance field grid
  cv::Mat distance_image_norm_;
  cv::normalize(distance_field_obstacle_image_, distance_image_norm_, 1, 0, cv::NORM_MINMAX);
  cv::imshow("original", binary_image_);
  cv::imshow("distances", distance_image_norm_);

  static const double sec = 1000;
  const double wait = 30 * sec;

  cv::waitKey(wait);
  cv::destroyAllWindows();
  */

  // convert opencv to distance_map_msgs
  matToDistanceFieldGrid(distance_field_obstacle_image_,
                         cost_map->getResolution(),
                         *field_obstacles_);

  return true;
}

} /* namespace distmap */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(distmap::DistanceMapOpencv, distmap::DistanceMapBase);
