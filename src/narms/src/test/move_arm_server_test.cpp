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
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <narms_utils.h>
#include <narms/target_pose.h>


int main(int argc, char*argv[]){


	ros::init(argc, argv, "pr2_planning_test");
	ros::NodeHandle nh;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	tf::TransformBroadcaster tf_broadcaster;
	// Visualization markers
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "ik_goals", 0 );
	
	// IK stuff
	moveit_msgs::GetPositionIK ik_srv;
	ros::ServiceClient compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	// PR2 Specific Messages
	std::string pr2_planning_group("pr2_right_arm");
	
	ros::ServiceClient pr2_move_arm_server = nh.serviceClient<narms::target_pose>("pr2_move_arm_server");
	
	
	// Roman Specific Messages
	std::string roman_planning_group("roman_right_arm");
	ros::ServiceClient roman_move_arm_server = nh.serviceClient<narms::target_pose>("roman_move_arm_server");

	double x_scale, y_scale, z_scale;
	ros::param::get("object_x_scale", x_scale);
	ros::param::get("object_y_scale", y_scale);
	ros::param::get("object_z_scale", z_scale);

	// Add a visualization marker for shits and giggles**************************
	visualization_msgs::Marker viz_marker;
	viz_marker.header.frame_id = "world";
	viz_marker.id = 0;
	viz_marker.type = visualization_msgs::Marker::CUBE;
	viz_marker.action = visualization_msgs::Marker::ADD;
	viz_marker.scale.x = x_scale;
	viz_marker.scale.y = y_scale;
	viz_marker.scale.z = z_scale;
	viz_marker.color.a = 1.0;
	viz_marker.color.r = 1.0;

	bool handoff_success = false;
	double sampling_variance_inflation_m = 0.0;
	double sampling_variance_inflation_step_m = 0.01;

	geometry_msgs::Pose pr2_pose;
	geometry_msgs::Pose roman_pose;

	for(int i = 0; i < 500; i++)
	{
		if(!ros::ok()) {return 0;}

		geometry_msgs::Pose randPose = generateRandomPose();

		tf::StampedTransform randTf(geoPose2Transform(randPose), ros::Time::now(), "world_link", "object");

		getGraspingPoses(randTf, pr2_pose, roman_pose);

		tf_broadcaster.sendTransform(tf::StampedTransform(geoPose2Transform(pr2_pose), ros::Time::now(), "world_link", "pr2_grasp"));
		tf_broadcaster.sendTransform(tf::StampedTransform(geoPose2Transform(roman_pose), ros::Time::now(), "world_link", "roman_grasp"));
		tf_broadcaster.sendTransform(randTf);

		// vizualize pose
		viz_marker.pose = randPose;
		viz_marker.header.stamp = ros::Time::now();
		ik_vis_pub.publish(viz_marker);

		getIKServerRequest(pr2_pose, pr2_planning_group, ik_srv);

		int n_reached = 0;

		// PR2 IK ***********************************************************************************

		ros::WallTime t0 = ros::WallTime::now();
		bool ik_suc = compute_ik.call(ik_srv);
		ros::WallTime t1 = ros::WallTime::now();
		std::cout << "pr2 compute_IK call time: " << (t1-t0).toSec() << "\n";

		if(ik_suc){
			if(ik_srv.response.error_code.val == 1){
				std::cout << "PR2 can reach goal!\n";
				n_reached++;
			}
			
		}else{
			std::cout << "Could not call PR2 IK service\n";
		}

		// Roman IK **********************************************************************************

		getIKServerRequest(roman_pose, roman_planning_group, ik_srv);

		t0 = ros::WallTime::now();
		ik_suc = compute_ik.call(ik_srv);
		t1 = ros::WallTime::now();
		std::cout << "roman compute_IK call time: " << (t1-t0).toSec() << "\n";


		if(ik_suc){
			if(ik_srv.response.error_code.val == 1){
				std::cout << "ROMAN can reach goal!\n";
				n_reached++;
			}
		}	else {
			std::cout << "Could not call ROMAN IK service\n";
		}

		// Planning and Execution ******************************************************************

	if (n_reached >= 2){
		// Solution is feasible for both PR2 and ROMAN, 
		// Now plan and execute
		int n_planned = 0;

		// PR2 planning and execution
		narms::target_pose pr2_mas_request;
		pr2_mas_request.request.pose = pr2_pose;

		ros::WallTime t2 = ros::WallTime::now();
		pr2_move_arm_server.call(pr2_mas_request);
		ros::WallTime t3 = ros::WallTime::now();
		std::cout << "pr2 move_arm_server call time: " << (t3-t2).toSec() << "\n";

		if(pr2_mas_request.response.result == true){
			std::cout << "PR2 plan found!\n";
			n_planned ++;
		}

		// Roman planning and execution
		narms::target_pose roman_mas_request;
		roman_mas_request.request.pose = roman_pose;

		t2 = ros::WallTime::now();
		roman_move_arm_server.call(roman_mas_request);
		t3 = ros::WallTime::now();
		std::cout << "roman move_arm_server call time: " << (t3-t2).toSec() << "\n";


		if(roman_mas_request.response.result == true)
		{
			std::cout << "ROMAN Plan executed!\n";
			n_planned++;
		}



		if(n_planned >= 2){
			std::cout << "\nBOTH ARM REACHED OBJECT\n";
			printPose(randPose);
			ros::Duration(2).sleep();
		}
	}
	
	}

return 0;
}