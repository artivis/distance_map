#include <distance_map_core/conversion.h>

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>

namespace distmap {

class DistanceMapImarker {
public:

  DistanceMapImarker() = default;
  virtual ~DistanceMapImarker() = default;

  void initialize();

  void process(const distance_map_msgs::DistanceFieldGridConstPtr dist_grid);

protected:

  bool configured_ = false; /*!< @brief Whether the node is configured. */

  std::shared_ptr<DistanceFieldGrid> dist_grid_ptr_;

  ros::NodeHandle private_nh_ = ros::NodeHandle("~");

  ros::Subscriber grid_sub_;

  ros::Publisher markers_pub_;

  std::string global_frame_ = "map";

  visualization_msgs::Marker gradient_marker_;
  visualization_msgs::Marker distance_marker_;
  visualization_msgs::MarkerArray distance_map_markers_;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

  void initialize_markers();
};

void DistanceMapImarker::initialize_markers()
{
  std_msgs::ColorRGBA color;
  color.a = 1; // Red
  color.r = 1;
  color.g = 0;
  color.b = 0;

  geometry_msgs::Vector3 scale;
  scale.x = 0.4;
  scale.y = 0.03;
  scale.z = 0.03;

  gradient_marker_.header.frame_id = global_frame_;
  gradient_marker_.ns       = "gradient";
  gradient_marker_.type     = visualization_msgs::Marker::ARROW;
  gradient_marker_.lifetime = ros::Duration(0);
  gradient_marker_.id       = 0;
  gradient_marker_.action   = visualization_msgs::Marker::MODIFY;
  gradient_marker_.color    = color;
  gradient_marker_.scale    = scale;

  scale.x = 0.1;
  scale.y = 0.1;
  scale.z = 0.15;
  color.r = 0; // Black

  distance_marker_.header.frame_id = global_frame_;
  distance_marker_.ns       = "distance";
  distance_marker_.type     = visualization_msgs::Marker::TEXT_VIEW_FACING;
  distance_marker_.lifetime = ros::Duration(0);
  distance_marker_.id       = 1;
  distance_marker_.action   = visualization_msgs::Marker::MODIFY;
  distance_marker_.color    = color;
  distance_marker_.scale    = scale;
}

void DistanceMapImarker::initialize()
{
  dist_grid_ptr_ = std::make_shared<DistanceFieldGrid>(DistanceFieldGrid::Dimension(5,5),
                                                       1,
                                                       DistanceFieldGrid::Origin());

  initialize_markers();

  markers_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("distance_map_markers", 1);
  grid_sub_ = private_nh_.subscribe("/distance_map_node/distance_field_obstacles", 1, &DistanceMapImarker::process, this);

  marker_server_ = boost::make_shared<interactive_markers::InteractiveMarkerServer>("distance_map_imarker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker imarker;
  imarker.header.frame_id = global_frame_;
  imarker.header.stamp = ros::Time::now();
  imarker.name = "imarker";
  imarker.pose.position.x = 0;
  imarker.pose.position.y = 0;
  imarker.pose.position.z = 0.1;
  imarker.pose.orientation.w = 1.0f;
  imarker.scale = 1;

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(control);

  marker_server_->insert(imarker);

  // Visualize distance field (distance & inverse gradient)

  auto imarker_cb =
      [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
  {
    if (dist_grid_ptr_ != nullptr)
    {
      const double d = dist_grid_ptr_->atPositionSafe(
            feedback->pose.position.x, feedback->pose.position.y, false);

      const auto g = dist_grid_ptr_->gradientAtPositionSafe(
            feedback->pose.position.x, feedback->pose.position.y, false);

      if (distance_map_markers_.markers.empty())
      {
        gradient_marker_.action = visualization_msgs::Marker::ADD;
        distance_map_markers_.markers.push_back(gradient_marker_);

        distance_marker_.action = visualization_msgs::Marker::ADD;
        distance_map_markers_.markers.push_back(distance_marker_);
      }

      const double angle = std::atan2(g.dy, g.dx);

      geometry_msgs::Pose pose;
      pose.position.x = feedback->pose.position.x;
      pose.position.y = feedback->pose.position.y;
      pose.position.z = 0.05;
      pose.orientation = tf::createQuaternionMsgFromYaw(angle);
      distance_map_markers_.markers[0].pose = pose;

      pose.position.z = 0.2;
      distance_map_markers_.markers[1].pose = pose;
      distance_map_markers_.markers[1].text = std::to_string(d);

      markers_pub_.publish(distance_map_markers_);

      ROS_INFO_STREAM("At position [" << feedback->pose.position.x << ","
                      << feedback->pose.position.y << "] distance is " << d
                      << " gradient is [" << g.dx << "," << g.dy << "]");
    }
  };

  marker_server_->setCallback(imarker.name, imarker_cb);

  marker_server_->applyChanges();

  ROS_INFO("Subscribed to %s", grid_sub_.getTopic().c_str());
}

void DistanceMapImarker::process(const distance_map_msgs::DistanceFieldGridConstPtr dist_grid)
{
  if (dist_grid == nullptr)
  {
    ROS_ERROR("Input nav_msgs::OccupancyGridConstPtr is nullptr !");
    return;
  }

  if (global_frame_ != dist_grid->header.frame_id)
  {
    global_frame_ = dist_grid->header.frame_id;
    initialize_markers();
  }

  ROS_ERROR("FROM_MSG 0");

  *dist_grid_ptr_ = distmap::fromMsg(*dist_grid);

  ROS_ERROR("FROM_MSG 1");
}

} /* namespace distmap */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_map_imarker");

  distmap::DistanceMapImarker node;

  node.initialize();

  ros::spin();

  return EXIT_SUCCESS;
}
