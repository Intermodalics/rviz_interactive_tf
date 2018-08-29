/*
 * Copyright (c) 2016, Lucas Walter
 *
 * Create a tf in rviz with a specified parent and name,
 * drag it around and rotate it with interactive marker controls.
 */

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <xmlrpcpp/XmlRpc.h>

void testFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{}
class InteractiveTf
{
  ros::NodeHandle nh_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void processFeedback(unsigned ind, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  visualization_msgs::InteractiveMarker int_marker_;

  geometry_msgs::TransformStamped transform_;
  tf2_ros::StaticTransformBroadcaster br_;
  bool lock_rotate_xy_;
  void updateTf(int);

public:
  InteractiveTf();
  ~InteractiveTf();
};

InteractiveTf::InteractiveTf() :
  lock_rotate_xy_(false)
{
  server_.reset(new interactive_markers::InteractiveMarkerServer("interactive_tf"));
  transform_.header.frame_id = "map";
  transform_.child_frame_id = "interactive_tf";
  transform_.transform.rotation.w = 1.0;

  ros::param::get("~parent_frame", transform_.header.frame_id);
  if (ros::param::has("~header/frame_id")) {
    ros::param::get("~header/frame_id", transform_.header.frame_id);
  }
  ros::param::get("~frame", transform_.child_frame_id);
  if (ros::param::has("~child_frame_id")) {
    ros::param::get("~child_frame_id", transform_.child_frame_id);
  }
  ros::param::get("~lock_rotate_xy", lock_rotate_xy_);

  // load initial pose from parameter server
  XmlRpc::XmlRpcValue initial_transform;
  if (ros::param::get("~transform", initial_transform)) {
    try {
      transform_.transform.translation.x = initial_transform["translation"]["x"];
      transform_.transform.translation.y = initial_transform["translation"]["y"];
      transform_.transform.translation.z = initial_transform["translation"]["z"];
      transform_.transform.rotation.w = initial_transform["rotation"]["w"];
      transform_.transform.rotation.x = initial_transform["rotation"]["x"];
      transform_.transform.rotation.y = initial_transform["rotation"]["y"];
      transform_.transform.rotation.z = initial_transform["rotation"]["z"];
    } catch(XmlRpc::XmlRpcException &e) {
      ROS_ERROR("Invalid transform parameter: %s", e.getMessage().c_str());
    }
  }

  int_marker_.header.frame_id = transform_.header.frame_id;
  // http://answers.ros.org/question/262866/interactive-marker-attached-to-a-moving-frame/
  // putting a timestamp on the marker makes it not appear
  // int_marker_.header.stamp = ros::Time::now();
	int_marker_.name = "interactive_tf";
	int_marker_.description = "control a tf with 6dof";
  int_marker_.pose.position.x = transform_.transform.translation.x;
  int_marker_.pose.position.y = transform_.transform.translation.y;
  int_marker_.pose.position.z = transform_.transform.translation.z + 0.01;
  int_marker_.pose.orientation.w = transform_.transform.rotation.w;
  int_marker_.pose.orientation.x = transform_.transform.rotation.x;
  int_marker_.pose.orientation.y = transform_.transform.rotation.y;
  int_marker_.pose.orientation.z = transform_.transform.rotation.z;

  {
  visualization_msgs::InteractiveMarkerControl control;

	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
  if (!lock_rotate_xy_) {
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
  }
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	int_marker_.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
  if (!lock_rotate_xy_) {
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
  }
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	int_marker_.controls.push_back(control);
  }

	server_->insert(int_marker_);
  // Can't seem to get rid of the 0, _1 parameter
	server_->setCallback(int_marker_.name,
      boost::bind(&InteractiveTf::processFeedback, this, 0, _1));
	// server_->setCallback(int_marker_.name, testFeedback);

  server_->applyChanges();

  updateTf(0);
}

InteractiveTf::~InteractiveTf()
{
  server_.reset();
}

void InteractiveTf::updateTf(int)
{
  transform_.header.stamp = ros::Time::now();
  br_.sendTransform(transform_);
}

void InteractiveTf::processFeedback(
    unsigned ind,
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM(feedback->header.frame_id);
  transform_.transform.translation.x = feedback->pose.position.x;
  transform_.transform.translation.y = feedback->pose.position.y;
  transform_.transform.translation.z = feedback->pose.position.z - 0.01;
  transform_.transform.rotation.w = feedback->pose.orientation.w;
  transform_.transform.rotation.x = feedback->pose.orientation.x;
  transform_.transform.rotation.y = feedback->pose.orientation.y;
  transform_.transform.rotation.z = feedback->pose.orientation.z;
  ROS_DEBUG_STREAM(feedback->control_name);
  ROS_DEBUG_STREAM(feedback->event_type);
  ROS_DEBUG_STREAM(feedback->mouse_point);

  updateTf(0);

	// TODO(lucasw) all the pose changes get handled by the server elsewhere?
	server_->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_tf");
  InteractiveTf interactive_tf;
  ros::spin();
  return 0;
}
