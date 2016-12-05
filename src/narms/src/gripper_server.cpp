#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

#include <narms/gripper_command.h>

class GripperServer{
public:
	double pr2_value_;
	double roman_value_;
	ros::NodeHandle nh_;
	ros::Publisher gripper_pub_;
	sensor_msgs::JointState joint_states_;
	ros::ServiceServer service_;

	GripperServer(){
		

		pr2_value_ = 0.55;
		roman_value_ = 0.0;

		joint_states_.header.frame_id = "world_link";
		joint_states_.header.stamp = ros::Time::now();
		joint_states_.position = {pr2_value_, roman_value_};
		joint_states_.name = {
			"pr2_r_gripper_l_finger_joint",
			"roman_limb_right_finger_middle_joint_1"
			};

		gripper_pub_ = nh_.advertise<sensor_msgs::JointState>("move_group/fake_controller_joint_states", 10);
		service_ = nh_.advertiseService("gripper_command_server", &GripperServer::handleRequest, this);

		gripper_pub_.publish(joint_states_);

	};

	bool handleRequest(narms::gripper_command::Request &req, narms::gripper_command::Response &res)
	{
		/*
			float32 pr2_command
			float32 roman_command
			---
			bool result
		*/
		if(req.pr2_command != -1){ pr2_value_ = req.pr2_command;}
		if(req.roman_command != -1) {roman_value_ = req.roman_command;}

		joint_states_.header.stamp = ros::Time::now();
		joint_states_.position = {pr2_value_, roman_value_};

		gripper_pub_.publish(joint_states_);

		res.result = true;
		return true;

	}
};

int main(int argc, char*argv[]){
	ros::init(argc, argv, "gripper_server");

	GripperServer server;
	ROS_INFO("gripper server online, awaiting fake gripper states");
	while (ros::ok())
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	return 0;
}