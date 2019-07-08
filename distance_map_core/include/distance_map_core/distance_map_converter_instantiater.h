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

#ifndef _DISTANCE_MAP_CORE_DISTANCE_MAP_INSTANTIATER_H_
#define _DISTANCE_MAP_CORE_DISTANCE_MAP_INSTANTIATER_H_

#include <pluginlib/class_loader.h>

#include "distance_map_core/singleton.h"
#include "distance_map_core/distance_map_converter_base.h"

namespace {

class DistanceMapConverterInstantiater final
{
public:

  DistanceMapConverterInstantiater()  = default;
  ~DistanceMapConverterInstantiater() = default;

  // Not copyable
  LaserOdometryInstanDistanceMapInstantiatertiater(DistanceMapConverterInstantiater&) = delete;
  void operator=(DistanceMapConverterInstantiater&)                                   = delete;

  distmap::DistanceMapPtr instantiate_impl(const std::string& distance_map_type)
  {
    distmap::DistanceMapPtr dist_map_ptr;
    bool configured = false;

    try
    {
      dist_map_ptr = loader.createInstance(distance_map_type);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason.\n\tError: %s", ex.what());
    }

    if (dist_map_ptr == nullptr)
    {
      ROS_ERROR_STREAM("Error creating distance map: "
                       << distance_map_type);
    }
    else
    {
      ROS_DEBUG_STREAM("Succes creating distance map: "
                       << distance_map_type);

      dist_map_ptr->setType(distance_map_type);
      configured = dist_map_ptr->configure();

      if (!configured)
      {
        ROS_ERROR_STREAM("Something went wrong while configuring pluging : "
                         << distance_map_type);
      }
    }

    return dist_map_ptr;
  }

protected:

  pluginlib::ClassLoader<distmap::DistanceMapConverterBase> loader =
    {"distance_map_core","distmap::DistanceMapConverterBase"};
};

} // namespace

namespace distmap {

namespace detail {
using Instantiater = details::Singleton<DistanceMapConverterInstantiater>;
} // namespace detail

inline DistanceMapPtr make_distance_mapper(const std::string& distance_map_type)
{
  return detail::Instantiater::get().instantiate_impl(distance_map_type);
}

} // namespace distmap

#endif // _DISTANCE_MAP_CORE_DISTANCE_MAP_INSTANTIATER_H_
