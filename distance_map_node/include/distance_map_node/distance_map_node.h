#ifndef _DISTANCE_MAP_NODE_DISTANCE_MAP_NODE_H_
#define _DISTANCE_MAP_NODE_DISTANCE_MAP_NODE_H_

#include <ros/ros.h>

namespace distmap {

/**
 * Forward declaration
 */
class DistanceMapBase;

class DistanceMapNode {
public:

  DistanceMapNode() = default;
  virtual ~DistanceMapNode() = default;

  void process();

protected:

  bool configured_ = false; /*!< @brief Whether the node is configured. */

  std::shared_ptr<DistanceMapBase> dist_map_;

  ros::NodeHandle nh_;

  ros::Subscriber map_sub_;

  ros::Publisher field_obstacles__pub_, field_unknowns_pub_;
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_NODE_DISTANCE_MAP_NODE_H_ */
