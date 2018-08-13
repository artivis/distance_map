#ifndef _DISTANCE_MAP_CORE_CONVERSION_H_
#define _DISTANCE_MAP_CORE_CONVERSION_H_

#include <distance_map_msgs/DistanceFieldGrid.h>
#include "distance_map_core/distance_field_grid.h"

#include <tf/tf.h>

namespace distmap {

distance_map_msgs::DistanceFieldGrid toMsg(const DistanceFieldGrid& map)
{
  distance_map_msgs::DistanceFieldGrid msg;

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

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_CONVERSION_H_ */
