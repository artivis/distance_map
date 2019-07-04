/*
 * Copyright 2019 Jeremie Deray
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
 * Author: Jeremie Deray
 */

#ifndef _DISTANCE_MAP_DEADRECK_DISTANCE_MAP_DEADRECK_H_
#define _DISTANCE_MAP_DEADRECK_DISTANCE_MAP_DEADRECK_H_

#include <distance_map_core/distance_map_converter_base.h>

namespace distmap {

class DistanceMapDeadReck : public DistanceMapConverterBase
{
public:

  DistanceMapDeadReck()  = default;
  ~DistanceMapDeadReck() = default;

  bool processImpl(const nav_msgs::OccupancyGridConstPtr occ_grid) override;
  bool processImpl(const costmap_2d::Costmap2D* cost_map) override;

protected:

  DistanceMap::Dimension previous_dim_ = DistanceMap::Dimension(1,1);
  std::unique_ptr<unsigned char[]> img_ptr_;
  std::unique_ptr<float[]> distance_field_ptr_;

  //bool configureImpl() override { return true; };
};

} /* namespace distmap */

#endif /* _DISTANCE_MAP_DEADRECK_DISTANCE_MAP_DEADRECK_H_ */
