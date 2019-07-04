#ifndef _DISTANCE_MAP_CORE_CONVERSION_H_
#define _DISTANCE_MAP_CORE_CONVERSION_H_

#include <distance_map_msgs/DistanceMap.h>
#include "distance_map_core/distance_map.h"

#include <tf/tf.h>

namespace distmap {

distance_map_msgs::DistanceMap toMsg(const DistanceMap& map)
{
  distance_map_msgs::DistanceMap msg;

  msg.info.width  = map.getDimension().width;
  msg.info.height = map.getDimension().height;
  msg.info.resolution = map.getResolution();
  msg.info.origin.position.x = map.getOrigin().x;
  msg.info.origin.position.y = map.getOrigin().y;
  msg.info.origin.position.z = 0;
  msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(map.getOrigin().yaw);

  const std::size_t num_elem = map.getDimension().width * map.getDimension().height;

  msg.data.reserve(num_elem);
  msg.data.insert(msg.data.end(), map.data(), map.data()+num_elem);

  return msg;
}

DistanceMap fromMsg(const distance_map_msgs::DistanceMap& msg)
{
  DistanceMap grid(DistanceMap::Dimension(msg.info.width, msg.info.height),
                         msg.info.resolution,
                         DistanceMap::Origin(msg.info.origin.position.x,
                                                   msg.info.origin.position.y,
                                                   tf::getYaw(msg.info.origin.orientation)));
  std::copy(msg.data.data(),
                 msg.data.data() + (msg.info.width * msg.info.height),
                 grid.data());
  return grid;
}

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_CONVERSION_H_ */
