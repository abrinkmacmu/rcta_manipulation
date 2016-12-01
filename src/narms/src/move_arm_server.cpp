#include <ros/ros.h>
#include <ros/console.h>
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


class MoveArmServer {
public:

	moveit::planning_interface::MoveGroup group;
	moveit::planning_interface::PlanningSceneInterface PSI;
	ros::AsyncSpinner spinner;
	ros::ServiceServer service;
	ros::NodeHandle nh;
	tf::StampedTransform pr2_tf;


	MoveArmServer(std::string group_name, std::string prefix):
	nh(),
	group(group_name),
	spinner(2)
	{

		tf::TransformListener listener;
		ros::Duration(1.0).sleep(); // wait for tf listener to start up
		listener.lookupTransform(group.getPoseReferenceFrame(),"world", ros::Time(0),pr2_tf);
		
		service = nh.advertiseService(prefix + "_move_arm_server", &MoveArmServer::handleRequest,this);
		
		ROS_INFO("Group Pose Reference Frame: %s" , group.getPoseReferenceFrame().c_str());
		//group.setPoseReferenceFrame("pr2/odom_combined");
		spinner.start();

		//std::cout << "Sanity Check the robot model\n";
		//robot_model::RobotModelConstPtr rm(group.getRobotModel());
		//rm->printModelInfo(std::cout);
	}

	void updateCollisionObjects()
	{
		double shelf_x, shelf_y, shelf_z;
		ros::param::get("shelf_x", shelf_x);
		ros::param::get("shelf_y", shelf_y);
		ros::param::get("shelf_z", shelf_z);

		double shelf_box_x, shelf_box_y, shelf_box_z;
		ros::param::get("shelf_box_x", shelf_box_x);
		ros::param::get("shelf_box_y", shelf_box_y);
		ros::param::get("shelf_box_z", shelf_box_z);

		geometry_msgs::Pose shelf_pose = generatePose(shelf_x, shelf_y, shelf_z, 0, 0, 0);

		moveit_msgs::CollisionObject CO;
		CO = createCollisionBox(shelf_pose, "shelf", "world_link", shelf_box_x, shelf_box_y, shelf_box_z);

		std::vector<moveit_msgs::CollisionObject> vecCollisionObjects;
		vecCollisionObjects.push_back(CO);
		PSI.addCollisionObjects(vecCollisionObjects);
	}


	bool handleRequest(narms::target_pose::Request &req, narms::target_pose::Response &res)
	{

		updateCollisionObjects();
		printPose(req.pose);
		//geometry_msgs::Pose rand_pose = group.getRandomPose().pose;
		//printPose(rand_pose);
		group.setStartState(*(group.getCurrentState()) );
		group.setPlannerId("RRTkConfigDefault");
		group.setPlanningTime(10); // sec
		//group.setPoseTarget(convertPoseViaTransform(req.pose, pr2_tf));
		group.setPoseTarget(req.pose);
		//group.setPoseReferenceFrame("pr2/r_shoulder_pan_link");
		//group.setPositionTarget(pose.position.x, pose.position.y, pose.position.z);
		moveit::planning_interface::MoveGroup::Plan plan;
		bool suc = group.plan(plan);
		if(suc && req.execute_plan)
		{
			ROS_INFO("MAS: plan found, executing now");
			group.execute(plan);
		}
		res.result = suc;
		return true;
	}
};


/** MAIN Run PR2 Server **/

int main(int argc, char*argv[]){
	ros::init(argc, argv, "move_arm_server");
	ros::NodeHandle ph("~");
	std::string group_name;
	std::string prefix;

	if (ph.hasParam("group_name") && ph.hasParam("prefix") ){

		ph.getParam("group_name", group_name);
		ph.getParam("prefix", prefix);
	} else {
		ROS_ERROR("Need parameters 'group_name' and 'prefix' to run move_arm_server");
		return 0;
	}

	MoveArmServer server(group_name, prefix);
	ROS_INFO("%smove_arm_server online, awaiting pose targets", prefix.c_str());
	while (ros::ok())
	{
		ros::Duration(0.1).sleep();
		ros::spinOnce();
	}
	return 0;
}
