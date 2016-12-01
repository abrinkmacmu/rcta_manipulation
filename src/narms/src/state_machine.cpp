
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <moveit_msgs/GetPositionIK.h>
#include <tf/transform_broadcaster.h>

#include <narms/target_pose.h>
#include <narms/gripper_command.h>

#include "narms_utils.h"
#include "states.h"



narms::gripper_command getGripperSrv(std::string robot, int state)
{
	narms::gripper_command srv;
	if(robot.compare("pr2")==0){
		if(state == 1){
			srv.request.pr2_command = 0.55;
			srv.request.roman_command = -1;
		}else{
			srv.request.pr2_command = 0.25;
			srv.request.roman_command = -1;
		}
	}


	if(robot.compare("roman")==0){
		if(state == 1){
			srv.request.pr2_command = -1;
			srv.request.roman_command = 0;
		}else{
			srv.request.pr2_command = -1;
			srv.request.roman_command = 0.68;
		}
	}

	return srv;

}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle nh;
	bool success;

	geometry_msgs::Pose startPose;
	geometry_msgs::Pose goalPose;
	geometry_msgs::Pose handoffPose;

	startPose = getPoseFromRosParamRPYDegrees("start_pose");
	goalPose =  getPoseFromRosParamRPYDegrees("goal_pose");

	printPose(startPose);
	printPose(goalPose);

	tf::TransformBroadcaster tf_broadcaster;
	for(int i = 0; i < 3; i++){
		// I do not know why I have to loop over this several times to get this to work!
		visualizeObjectPose(startPose, "start_pose_");
		visualizeObjectPose(goalPose, "goal_pose_");
	}

	narms::target_pose mas_srv;
	narms::gripper_command gripper_srv;

	ros::ServiceClient startRobotMAS;
	ros::ServiceClient goalRobotMAS;
	ros::ServiceClient gripperServer = nh.serviceClient<narms::gripper_command>("gripper_command_server");
	ros::ServiceClient computeIK = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	std::string startRobot;
	std::string goalRobot;


	// State 1 *************************************************************
	success = checkStartandGoalIK(startPose, goalPose, computeIK, startRobot, goalRobot);
	if(!success) { ROS_ERROR("Could not find acceptable start and goal IK solutions for robot system"); return 0;}
	ROS_INFO("Start robot: %s", startRobot.c_str());
	ROS_INFO("Goal robot: %s", goalRobot.c_str());


	geometry_msgs::Pose startRobotPose;
	geometry_msgs::Pose goalRobotPose;
	geometry_msgs::Pose pr2_pose;
	geometry_msgs::Pose roman_pose;
	getGraspingPoses(geoPose2Transform(startPose), pr2_pose, roman_pose);

	if( startRobot.compare("pr2") == 0){   
		startRobotMAS = nh.serviceClient<narms::target_pose>("pr2_move_arm_server");
		startRobotPose = pr2_pose;
	}
	if( startRobot.compare("roman") == 0){ 
		startRobotMAS = nh.serviceClient<narms::target_pose>("roman_move_arm_server");
		startRobotPose = roman_pose;
	}

	getGraspingPoses(geoPose2Transform(goalPose), pr2_pose, roman_pose);

	if( goalRobot.compare("pr2") == 0){		
		goalRobotMAS = nh.serviceClient<narms::target_pose>("pr2_move_arm_server");
		goalRobotPose = pr2_pose;
	}
	if( goalRobot.compare("roman") == 0){ 
		goalRobotMAS = nh.serviceClient<narms::target_pose>("roman_move_arm_server");
		goalRobotPose = roman_pose;
	}




	// State 2**********************************************************
	
	success = computeSampledHandoffPose(startPose, goalPose, startRobot, goalRobot, handoffPose);
	if(!success) { ROS_ERROR("Could not find handoff pose in reasonable timeframe"); return 0;}
	
	geometry_msgs::Pose startRobotHandoffPose;
	geometry_msgs::Pose goalRobotHandoffPose;

	getGraspingPoses(geoPose2Transform(handoffPose), pr2_pose, roman_pose);

	if( startRobot.compare("pr2") == 0){   startRobotHandoffPose = pr2_pose; }
	if( startRobot.compare("roman") == 0){ startRobotHandoffPose = roman_pose; }

	if( goalRobot.compare("pr2") == 0){		goalRobotHandoffPose = pr2_pose; }
	if( goalRobot.compare("roman") == 0){ goalRobotHandoffPose = roman_pose; }



	// Linear progression of service calls ******************************************************
	
	mas_srv.request.pose = startRobotPose; mas_srv.request.execute_plan = true;
	success = startRobotMAS.call(mas_srv);
	if(!mas_srv.response.result) { ROS_ERROR("Could not move start robot to pickup"); return 0;}	

	gripper_srv = getGripperSrv(startRobot, 0);
	gripperServer.call(gripper_srv);
	if(!gripper_srv.response.result) {ROS_ERROR("Could not command gripper"); return 0;}

	mas_srv.request.pose = startRobotHandoffPose; mas_srv.request.execute_plan = true;
	success = startRobotMAS.call(mas_srv);
	if(!mas_srv.response.result) { ROS_ERROR("Could not move start robot to handoff"); return 0;}	

	mas_srv.request.pose = goalRobotHandoffPose; mas_srv.request.execute_plan = true;
	success = goalRobotMAS.call(mas_srv);
	if(!mas_srv.response.result) { ROS_ERROR("Could not move goal robot to handoff"); return 0;}	

	gripper_srv = getGripperSrv(goalRobot, 0);
	gripperServer.call(gripper_srv);
	if(!gripper_srv.response.result) {ROS_ERROR("Could not command gripper"); return 0;}

	gripper_srv = getGripperSrv(startRobot, 1);
	gripperServer.call(gripper_srv);
	if(!gripper_srv.response.result) {ROS_ERROR("Could not command gripper"); return 0;}

	mas_srv.request.pose = goalRobotPose; mas_srv.request.execute_plan = true;
	success = goalRobotMAS.call(mas_srv);
	if(!mas_srv.response.result) { ROS_ERROR("Could not move goal robot to goal"); return 0;}

	gripper_srv = getGripperSrv(goalRobot, 1);
	gripperServer.call(gripper_srv);
	if(!gripper_srv.response.result) {ROS_ERROR("Could not command gripper"); return 0;}
	
}