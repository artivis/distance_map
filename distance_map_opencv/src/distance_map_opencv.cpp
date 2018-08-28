#include "distance_map_opencv/distance_map_opencv.h"
#include "distance_map_msgs/DistanceFieldGrid.h"

#include <ros/ros.h>
#include <costmap_2d/cost_values.h>

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
        cv_map.at<uchar>(rows, cols) = 127; //205?
      }
    }
  }

  return cv_map;
}

cv::Mat costMapToMat(const costmap_2d::Costmap2D& costmap)
{
  const unsigned int X = costmap.getSizeInCellsX(),
                     Y = costmap.getSizeInCellsY();
  cv::Mat cv_map(Y, X, CV_8UC1);
  unsigned int i;
  // Make a double loop over indexes and assign values
  unsigned char const* data = costmap.getCharMap();
  for (unsigned int rows = 0; rows < Y; ++rows) {
    for (unsigned int cols = 0; cols < X; ++cols) {
      i = cols + (Y - rows - 1) * X;
      if (data[i] == costmap_2d::INSCRIBED_INFLATED_OBSTACLE or
          data[i] == costmap_2d::LETHAL_OBSTACLE) { // Obstacle
        cv_map.at<uchar>(rows, cols) = 0;
      } else if (data[i] == costmap_2d::NO_INFORMATION) { // UNKNOWN
        cv_map.at<uchar>(rows, cols) = 127;
      } else { // FREE_SPACE
        cv_map.at<uchar>(rows, cols) = 254;
      }
    }
  }

  return cv_map;
}

void matToDistanceFieldGrid(const cv::Mat& cv_map,
                            const std::size_t width,
                            const std::size_t height,
                            const double resolution,
                            distmap::DistanceFieldGrid &map)
{
  map.resize(width, height);
  map.setResolution(resolution);

  unsigned int i;
  for (unsigned int rows = 0; rows < height; ++rows) {
    for (unsigned int cols = 0; cols < width; ++cols) {
      i = cols + rows * width;
      map.data()[i] = static_cast<double>(cv_map.at<float>(rows, cols));
    }
  }
}

void matToDistanceFieldGrid(const cv::Mat& cv_map,
                            const nav_msgs::MapMetaData& map_metadata,
                            distmap::DistanceFieldGrid &map)
{
  matToDistanceFieldGrid(cv_map,
                         static_cast<std::size_t>(map_metadata.width),
                         static_cast<std::size_t>(map_metadata.height),
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
  cv::threshold(image_, binary_image_, 55, 255, cv::THRESH_BINARY);

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
                         occ_grid->info, *field_obstacles_);

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
  cv::threshold(image_, binary_image_, 55, 255, cv::THRESH_BINARY);

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
                         cost_map->getSizeInCellsX(),
                         cost_map->getSizeInCellsY(),
                         cost_map->getResolution(),
                         *field_obstacles_);

  return true;
}

} /* namespace distmap */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(distmap::DistanceMapOpencv, distmap::DistanceMapBase);
