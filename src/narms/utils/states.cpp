
#include <moveit_msgs/GetPositionIK.h>

#include "states.h"
#include "narms_utils.h"

bool checkStartandGoalIK(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, 
	ros::ServiceClient& computeIK,
	std::string& startRobot, std::string& goalRobot)
{

	bool startSuccess, goalSuccess = false;
	geometry_msgs::Pose pr2_pose;
	geometry_msgs::Pose roman_pose;
	moveit_msgs::GetPositionIK ik_srv;

	// Starting Robot

	getGraspingPoses(geoPose2Transform(startPose), pr2_pose, roman_pose);
	getIKServerRequest(roman_pose, "roman_right_arm", ik_srv);
	computeIK.call(ik_srv);
	if(ik_srv.response.error_code.val==1) {
		startRobot = "roman";
		startSuccess = true;
		ROS_INFO("StartPose reached");
	}else{
		getIKServerRequest(pr2_pose, "pr2_right_arm", ik_srv);
		computeIK.call(ik_srv);
		if(ik_srv.response.error_code.val == 1) {
			startRobot = "pr2";
			startSuccess = true;
			ROS_INFO("StartPose reached");
		}

	}


	// Goal Robot
	getGraspingPoses(geoPose2Transform(goalPose), pr2_pose, roman_pose);
	getIKServerRequest(pr2_pose, "pr2_right_arm", ik_srv);
	computeIK.call(ik_srv);
	if(ik_srv.response.error_code.val == 1) {
		goalRobot = "pr2";
		goalSuccess = true;
		ROS_INFO("GoalPose reached");
	}else{
		getIKServerRequest(roman_pose, "roman_right_arm", ik_srv);
		computeIK.call(ik_srv);
		if(ik_srv.response.error_code.val==1) {
			goalRobot = "roman";
			goalSuccess = true;
			ROS_INFO("GoalPose reached");
		}
	}

	if(startSuccess && goalSuccess){
		return true;
	}else{
		return false;
	}
}

bool computeSampledHandoffPose(geometry_msgs::Pose startPose, geometry_msgs::Pose goalPose, 
	std::string startRobot, std::string goalRobot, geometry_msgs::Pose& handoffPose)
{
	// ;(0.375207, 1.26025, 0.768803)   q[0.163179, 0.207972, -0.628604, 0.731421
	handoffPose.position.x = 0.375207;
	handoffPose.position.y = 1.26025;
	handoffPose.position.z = 0.768803;
	handoffPose.orientation.x = 0.163179;
	handoffPose.orientation.y = 0.207972;
	handoffPose.orientation.z = -0.628604;
	handoffPose.orientation.w = 0.731421;
	return true;
}