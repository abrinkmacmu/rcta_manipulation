#ifndef stow_arm_h
#define stow_arm_h

#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <hdt/MoveArmCommandAction.h>
#include <hdt/common/hdt_description/RobotModel.h>
#include <ros/ros.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/utils/utils.h>

bool cb_success = false;

struct StowPosition
{
	std::string name;
	std::vector<double> joint_positions;
};

bool extract_xml_value(
    XmlRpc::XmlRpcValue& value,
    StowPosition& stow_position)
{
    if (!value.hasMember("name") || !value.hasMember("joint_vector_degs")) {
        return false;
    }

    StowPosition tmp;
    bool success =
            msg_utils::extract_xml_value(value["name"], tmp.name) &&
            msg_utils::extract_xml_value(value["joint_vector_degs"], tmp.joint_positions);

    if (success) {
        stow_position = tmp;
    }
    return success;
}

void result_callback(
	const actionlib::SimpleClientGoalState& state,
	const hdt::MoveArmCommandResult::ConstPtr& result)
{
	if(result->success){
		ROS_INFO("Arm stowed!");
		cb_success = true;
	} else {
		ROS_WARN("...");
	}
}

enum MainResult
{
	SUCCESS = 0,
	FAILED_TO_STOW_ARM,
	FAILED_TO_RETRIEVE_ROBOT_DESCRIPTION,
	TIMED_OUT_WAITING_FOR_SERVER,
	TIMED_OUT_WAITING_FOR_RESULT,
	FAILED_TO_RETRIEVE_STOW_POSITIONS
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "stow_arm");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");

	std::string urdf_string;
	if (!nh.getParam("robot_description", urdf_string)) {
		ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
		exit(FAILED_TO_RETRIEVE_ROBOT_DESCRIPTION);
	}

	hdt::RobotModelPtr robot_model = hdt::RobotModel::LoadFromURDF(urdf_string);

	std::vector<StowPosition> stow_positions_;

	// read in stow positions
    	if (!msg_utils::download_param(ph, "stow_positions", stow_positions_)) {
        	ROS_ERROR("Failed to retrieve 'stow_positions' from the param server");
	        exit(FAILED_TO_RETRIEVE_STOW_POSITIONS);
    	}

	ROS_INFO("Stow Positions:");
	for (StowPosition& position : stow_positions_) {
		ROS_INFO("    %s: %s", position.name.c_str(), to_string(position.joint_positions).c_str());
		position.joint_positions = msg_utils::to_radians(position.joint_positions);
	}

	const std::string move_arm_command_action_name = "move_arm_command";
	actionlib::SimpleActionClient<hdt::MoveArmCommandAction> client(move_arm_command_action_name, true);

	if (!client.waitForServer(ros::Duration(10.0))) {
		exit(TIMED_OUT_WAITING_FOR_SERVER);
	}

	hdt::MoveArmCommandGoal move_arm_stow_goal;
	move_arm_stow_goal.type = hdt::MoveArmCommandGoal::JointGoal;
	move_arm_stow_goal.goal_joint_state.name = robot_model->joint_names();
	move_arm_stow_goal.octomap; // meh?
    	//include the attached object in the goal
    	move_arm_stow_goal.has_attached_object = false;
    	move_arm_stow_goal.execute_path = true;

	for(size_t i = 0; i < stow_positions_.size(); i++){
		ROS_INFO("Trying stow position %s: %s", stow_positions_[i].name.c_str(), to_string(stow_positions_[i].joint_positions).c_str());

		move_arm_stow_goal.goal_joint_state.position = stow_positions_[i].joint_positions;
    			//std::vector<double>(robot_model->joint_names().size(), 0);

		auto result_cb = boost::bind(result_callback, _1, _2);
		client.sendGoal(move_arm_stow_goal, result_cb);
		if (!client.waitForResult()) {
			exit(TIMED_OUT_WAITING_FOR_RESULT);
		}
		if(cb_success) return SUCCESS;
	}
	//if we got here all stow positions failed!
	ROS_WARN("All stow positions failed!");
	return FAILED_TO_STOW_ARM;
	//return SUCCESS;
}

#endif
