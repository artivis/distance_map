#ifndef _DISTANCE_MAP_CORE_CONVERSION_H_
#define _DISTANCE_MAP_CORE_CONVERSION_H_

#include <distance_map_msgs/DistanceFieldGrid.h>
#include "distance_map_core/distance_field_grid.h"

namespace distmap {

distance_map_msgs::DistanceFieldGrid toMsg(const DistanceFieldGrid& map)
{
  distance_map_msgs::DistanceFieldGrid msg;

  const std::size_t num_elem = map.getDimension().width * map.getDimension().height;

  msg.data.reserve(num_elem);
  msg.data.insert(msg.data.end(), map.data(), map.data()+num_elem);

  return msg;
}

} /* namespace distmap */

#endif /* _DISTANCE_MAP_CORE_CONVERSION_H_ */
