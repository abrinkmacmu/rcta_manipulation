#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <string>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include "narms_utils.h"

std::string g_robot;

void possessionCB(const std_msgs::String::ConstPtr& msg)
{
  g_robot = msg->data;
}

int main(int argc, char*argv[])
{
	ros::init(argc, argv, "object_node");
	ros::NodeHandle nh;
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "object", 0 );
	ros::Subscriber possessionSub = nh.subscribe("arm_possession", 1, possessionCB);

	visualization_msgs::Marker viz_marker;
	viz_marker.header.frame_id = "world";
	viz_marker.id = 0;
	viz_marker.type = visualization_msgs::Marker::CUBE;
	viz_marker.action = visualization_msgs::Marker::ADD;
	ros::param::get("object_x_scale", viz_marker.scale.x);
	ros::param::get("object_y_scale", viz_marker.scale.y);
	ros::param::get("object_z_scale", viz_marker.scale.z);

	viz_marker.color.a = 1.0;
	viz_marker.color.r = 1.0;

	geometry_msgs::Pose start_pose = getPoseFromRosParamRPYDegrees("start_pose");

	tf::TransformListener listener;
	tf::StampedTransform transform;

	while(ros::ok())
	{

		if( g_robot.compare("") == 0 || g_robot.compare("start")==0 ){
			viz_marker.pose = start_pose;

		}else if (g_robot.compare("pr2") == 0){
			
	    try{
	      listener.lookupTransform("world_link", "pr2/r_gripper_tool_frame", ros::Time(0), transform);
	    }catch(...){}

			getObjectPoseFromToolFrame(g_robot, transform, viz_marker.pose);
		}else if (g_robot.compare("roman") == 0){
			
	    try{
	      listener.lookupTransform("world_link", "roman/limb_right_tool0", ros::Time(0), transform);
	    }catch(...){}

			getObjectPoseFromToolFrame(g_robot, transform, viz_marker.pose);
		}

		ik_vis_pub.publish(viz_marker);

		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	return 0;
}