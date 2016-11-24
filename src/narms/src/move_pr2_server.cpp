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
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include <narms/target_pose.h>
#include <narms_utils.h>


class MovePR2Server {
public:

	moveit::planning_interface::MoveGroup group;
	
	ros::AsyncSpinner spinner;
	ros::ServiceServer service;
	ros::NodeHandle nh;
	tf::StampedTransform pr2_tf;

	MovePR2Server():
	nh(),
	group("right_arm"),
	spinner(2)
	{

		tf::TransformListener listener;
		ros::Duration(1.0).sleep(); // wait for tf listener to start up
		listener.lookupTransform("pr2/r_shoulder_pan_link","world", ros::Time(0),pr2_tf);
		
		service = nh.advertiseService("move_pr2_to_pose", &MovePR2Server::handleRequest,this);
		
		std::cout << "Group Pose Reference Frame: " << group.getPoseReferenceFrame() < "\n";
		//group.setPoseReferenceFrame("pr2/odom_combined");
		spinner.start();

		std::cout << "Sanity Check the robot model\n";
		robot_model::RobotModelConstPtr rm(group.getRobotModel());
		rm->printModelInfo(std::cout);
	}


	bool handleRequest(narms::target_pose::Request &req, narms::target_pose::Response &res)
	{
		std::cout << "New move PR2 pose request!\n";
		group.setStartState(*(group.getCurrentState()) );
		group.setPlannerId("RRTkConfigDefault");
		group.setPlanningTime(10); // sec
		group.setPoseTarget(convertPoseViaTransform(req.pose, pr2_tf));
		//group.setPoseTarget(req.pose);
		//group.setPoseReferenceFrame("pr2/r_shoulder_pan_link");
		//group.setPositionTarget(pose.position.x, pose.position.y, pose.position.z);
		std::cout << "Group set\n";
		moveit::planning_interface::MoveGroup::Plan plan;
		bool suc = group.plan(plan);
		if(suc)
		{
			std::cout  << "PR2 plan found, executing now...\n";
			group.execute(plan);
		}
		res.result = suc;
		return true;
	}
};


/** MAIN Run PR2 Server **/

int main(int argc, char*argv[]){
	ros::init(argc, argv, "move_pr2_server");
	//ros::NodeHandle nh;

	MovePR2Server server;
	while (ros::ok())
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	return 0;
}
