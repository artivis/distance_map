#include "distance_map_node/distance_map_node.h"

#include <distance_map_core/distance_map_base.h>
#include <distance_map_core/distance_map_instantiater.h>

namespace distmap {

} /* namespace distmap */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_map_node");

  distmap::DistanceMapNode node;

  ros::spin();

  return EXIT_SUCCESS;
}
