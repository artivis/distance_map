#include <distance_map_core/distance_map_converter_base.h>
#include <distance_map_core/conversion.h>
#include <distance_map_core/distance_map_converter_instantiater.h>

#include <ros/ros.h>

namespace distmap {

class DistanceMapNode {
public:

  DistanceMapNode() = default;
  virtual ~DistanceMapNode() = default;

  void initialize();

  void process(const nav_msgs::OccupancyGridConstPtr occ_grid);

protected:

  bool configured_ = false; /*!< @brief Whether the node is configured. */

  boost::shared_ptr<DistanceMapConverterBase> dist_map_ptr_;

  ros::NodeHandle private_nh_ = ros::NodeHandle("~");

  ros::Subscriber map_sub_;

  ros::Publisher field_obstacles_pub_, field_unknowns_pub_;
};

void DistanceMapNode::initialize()
{
  std::string distance_map_type;
  if (!private_nh_.getParam("distance_map_type", distance_map_type))
  {
    ROS_ERROR("No distance map type specified !");
    throw std::runtime_error("No distance map type specified !");
  }

  dist_map_ptr_ = make_distance_mapper(distance_map_type);

  if (dist_map_ptr_ != nullptr)
    configured_ = true;
  else
  {
    ROS_ERROR("Something went wrong.");
    throw std::runtime_error("Something went wrong.");
  }

  map_sub_ = private_nh_.subscribe("/map", 1, &DistanceMapNode::process, this);

  field_obstacles_pub_ = private_nh_.advertise<distance_map_msgs::DistanceMap>("distance_field_obstacles", 1, true);
//  field_unknowns_pub_  = private_nh_.advertise<distance_map_msgs::DistanceMap>("distance_field_unknowns",  1, true);

  ROS_INFO("Subscribed to %s", map_sub_.getTopic().c_str());
}

void DistanceMapNode::process(const nav_msgs::OccupancyGridConstPtr occ_grid)
{
  if (dist_map_ptr_ == nullptr)
  {
    ROS_ERROR("Reached callback but distance map ptr is nullptr !");
    throw std::runtime_error("Reached callback but distance map ptr is nullptr !");
  }

  if (occ_grid == nullptr)
  {
    ROS_ERROR("Input nav_msgs::OccupancyGridConstPtr is nullptr !");
    return;
  }

  if (dist_map_ptr_->process(occ_grid))
  {
    auto dist_grid_ptr = dist_map_ptr_->getDistanceFieldObstacle();

    assert(dist_grid_ptr != nullptr && "dist_grid_ptr == nullptr !");

    auto field_msg = distmap::toMsg(*dist_grid_ptr);

    field_msg.header = occ_grid->header;

    field_obstacles_pub_.publish(field_msg);
    //field_unknowns_pub_.publish(dist_map_ptr_->getDistanceFieldUnknown());
  }
  else
  {
    ROS_WARN("Could not process occupancy_grid !");
  }
}

} /* namespace distmap */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_map_node");

  distmap::DistanceMapNode node;

  node.initialize();

  ros::spin();

  return EXIT_SUCCESS;
}
