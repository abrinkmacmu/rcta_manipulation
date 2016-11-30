#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <string>

#include <cstdlib>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/RobotState.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/GetPositionIK.h>

#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <narms_utils.h>
#include <narms/target_pose.h>


int main(int argc, char*argv[]){


	ros::init(argc, argv, "pr2_planning_test");
	ros::NodeHandle nh;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "ik_goals", 0 );

	// PR2 Specific Messages
	std::string pr2_planning_group("pr2_right_arm");
	moveit_msgs::GetPositionIK ik_srv;
	ros::ServiceClient compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	ros::ServiceClient pr2_move_arm_server = nh.serviceClient<narms::target_pose>("pr2_move_arm_server");
	
	
	// Roman Specific Messages
	std::string roman_planning_group("roman_right_arm");


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

	bool handoff_sucess = false;
	double sampling_variance_inflation_m = 0.0;
	double sampling_variance_inflation_step_m = 0.01;

	for(int i = 0; i < 100; i++)
	{
		std::cout<<"\nSampled a point\n";
		geometry_msgs::Pose randPose = generateRandomPose();

		getIKServerRequest(randPose, pr2_planning_group, ik_srv);

		if(compute_ik.call(ik_srv))
		{
			//std::cout << "Error code: " << pr2_ik_srv.response.error_code.val << "\n";
			if(ik_srv.response.error_code.val == 1){
				std::cout << "PR2 can reach goal!\n";
			}

			narms::target_pose pr2_mas_request;
			pr2_mas_request.request.pose = randPose;
			pr2_move_arm_server.call(pr2_mas_request);
			if(pr2_mas_request.response.result == true){
				std::cout << "PR2 plan found!\n";
			}

		}else{
			std::cout << "Could not call PR2 IK service\n";
		}

		// Roman

		getIKServerRequest(randPose, roman_planning_group, ik_srv);

		if(compute_ik.call(ik_srv)){
			
			if(ik_srv.response.error_code.val == 1){
				std::cout << "ROMAN can reach goal!\n";
			}

		}	else {
		
			std::cout << "Could not call ROMAN IK service\n";
		
		}


		// vizualize pose
		viz_marker.pose = randPose;
		viz_marker.header.stamp = ros::Time::now();
		ik_vis_pub.publish(viz_marker);

		ros::Duration(.3).sleep();
	}

return 0;
}