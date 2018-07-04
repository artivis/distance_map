#ifndef _DISTANCE_MAP_NODE_DISTANCE_MAP_NODE_H_
#define _DISTANCE_MAP_NODE_DISTANCE_MAP_NODE_H_

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

namespace distmap {

/**
 * Forward declaration
 */
class DistanceMapBase;

class DistanceMapNode {
public:

  DistanceMapNode() = default;
  virtual ~DistanceMapNode() = default;

  void initialize();

  void process(const nav_msgs::OccupancyGridConstPtr occ_grid);

protected:

  bool configured_ = false; /*!< @brief Whether the node is configured. */

  boost::shared_ptr<DistanceMapBase> dist_map_ptr_;

  ros::NodeHandle private_nh_ = ros::NodeHandle("~");

  ros::Subscriber map_sub_;

  ros::Publisher field_obstacles_pub_, field_unknowns_pub_;
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_NODE_DISTANCE_MAP_NODE_H_ */
