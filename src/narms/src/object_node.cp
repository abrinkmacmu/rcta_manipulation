#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <string>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

std::string g_robot;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  g_robot = msg->data;
}

int main(int argc, char*argv[])
{
	ros::init(argc, argv, "object_node");
	ros::NodeHandle nh;
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "object", 0 );
	ros::Subscriber

	// Add a visualization marker for shits and giggles**************************
	visualization_msgs::Marker viz_marker;
	viz_marker.header.frame_id = "world";
	viz_marker.id = 0;
	viz_marker.type = visualization_msgs::Marker::CUBE;
	viz_marker.action = visualization_msgs::Marker::ADD;
	viz_marker.scale.x = .05;
	viz_marker.scale.y = .05;
	viz_marker.scale.z = .05;
	viz_marker.color.a = 1.0;
	viz_marker.color.r = 1.0;

	while(ros::ok())
	{



		if(g_robot.compare("")==0){
			viz_marker.pose
		}

		ik_vis_pub.publish(viz_marker);

		ros::spinOnce();
		ros::Duration(0.05);
	}