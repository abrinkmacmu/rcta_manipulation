
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
	std::string startRobot, std::string goalRobot, geometry_msgs::Pose& handoffPose,
	unsigned int num_of_handoff_samples,
	ros::ServiceClient& compute_ik,
	ros::ServiceClient& robot1_move_arm_server,
	ros::ServiceClient& robot2_move_arm_server)
{
	unsigned int handoffs_found = 0;
	moveit_msgs::GetPositionIK ik_srv_robot1;
	moveit_msgs::GetPositionIK ik_srv_robot2;

	if (startRobot == "roman")
	{
		getIKServerRequest(startPose, "roman_right_arm", ik_srv_robot1);
		getIKServerRequest(goalPose, "pr2_right_arm", ik_srv_robot2);
	}
	else
	{
		getIKServerRequest(startPose, "pr2_right_arm", ik_srv_robot1);
		getIKServerRequest(goalPose, "roman_right_arm", ik_srv_robot2);
	}
	
	// ros::ServiceClient compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

	// Collection of valid trajectories to be later used for evaluation
	std::vector<moveit_msgs::RobotTrajectory> robot1_trajectories;
	std::vector<moveit_msgs::RobotTrajectory> robot2_trajectories;

	tf::TransformBroadcaster tf_broadcaster;



	while(handoffs_found!=num_of_handoff_samples)
	{
		//continue sampling till you find a solution reachable by both arms
		if(!ros::ok()) {return 0;}

		geometry_msgs::Pose handoffPose = generateRandomPose(); // Replace with sampled pose

		tf::StampedTransform handoffTf(geoPose2Transform(handoffPose), ros::Time::now(), "world_link", "object");

		getGraspingPoses(handoffTf, startPose, goalPose);

		tf_broadcaster.sendTransform(tf::StampedTransform(geoPose2Transform(startPose), ros::Time::now(), "world_link", "pr2_grasp"));
		tf_broadcaster.sendTransform(tf::StampedTransform(geoPose2Transform(goalPose), ros::Time::now(), "world_link", "roman_grasp"));
		tf_broadcaster.sendTransform(handoffTf);

		// vizualize pose
		// viz_marker.pose = handoffPose;
		// viz_marker.header.stamp = ros::Time::now();
		// ik_vis_pub.publish(viz_marker);


		int n_reached = 0;

		// PR2 IK ***********************************************************************************

		ros::WallTime t0 = ros::WallTime::now();
		bool ik_suc = compute_ik.call(ik_srv_robot1);
		ros::WallTime t1 = ros::WallTime::now();
		std::cout << "Robot1 compute_IK call time: " << (t1-t0).toSec() << "\n";

		if(ik_suc){
			if(ik_srv_robot1.response.error_code.val == 1){
				std::cout << "Robot1 can reach goal!\n";
				n_reached++;
			}
			
			else
			{
				std::cout<< "FAILED!\n";
			}
			
		}else{
			std::cout << "Could not call Robot1 IK service\n";
		}

		// Roman IK **********************************************************************************

		// getIKServerRequest(goalPose, goalRobot, ik_srv);

		t0 = ros::WallTime::now();
		ik_suc = compute_ik.call(ik_srv_robot2);
		t1 = ros::WallTime::now();
		std::cout << "Robot2 compute_IK call time: " << (t1-t0).toSec() << "\n";


		if(ik_suc){
			if(ik_srv_robot2.response.error_code.val == 1){
				std::cout << "ROMAN can reach goal!\n";
				n_reached++;
			}
			else
			{
				std::cout<< "FAILED!\n";
			}
		}	else {
			std::cout << "Could not call ROMAN IK service\n";
		}

		// Planning and Execution ******************************************************************

	if (n_reached >= 2){
		// Solution is feasible for both PR2 and ROMAN, 
		// Get the trajectory for this point and evaluate later
		
		ROS_INFO("SAMPLING POINT FOUND..FINDING PLANS");		
		int n_planned = 0;

		// PR2 planning and execution
		narms::target_pose robot1_mas_request;
		robot1_mas_request.request.pose = startPose;
		robot1_mas_request.request.execute_plan=false;


		ros::WallTime t2 = ros::WallTime::now();
		robot1_move_arm_server.call(robot1_mas_request);
		ros::WallTime t3 = ros::WallTime::now();
		std::cout << "pr2 move_arm_server call time: " << (t3-t2).toSec() << "\n";

		if(robot1_mas_request.response.result == true){
			std::cout << "PR2 Trajectory Found!\n";
			n_planned ++;
		}
		else
		{
			std::cout << "PR2 Trajectory Not Found!\n";
		}

		// Roman planning and execution
		narms::target_pose robot2_mas_request;
		robot2_mas_request.request.pose = goalPose;
		robot2_mas_request.request.execute_plan=false;

		t2 = ros::WallTime::now();
		robot2_move_arm_server.call(robot2_mas_request);
		t3 = ros::WallTime::now();
		std::cout << "roman move_arm_server call time: " << (t3-t2).toSec() << "\n";


		if(robot2_mas_request.response.result == true)
		{
			std::cout << "ROMAN Trajectory Found!\n";
			n_planned++;
		}
		else
		{
			std::cout << "ROMAN Trajectory Not Found!\n";
		}

		if(n_planned >= 2){
			std::cout << "\nBOTH ARM REACHED OBJECT\n";
			printPose(handoffPose);
			ros::Duration(2).sleep();
			robot1_trajectories.push_back(robot1_mas_request.response.traj);
			robot2_trajectories.push_back(robot2_mas_request.response.traj);
			++handoffs_found;
			std::cout<<"\nhandoffs_found: "<<handoffs_found;
		}
	}

	}
	return true;
}


double score_trajectory(moveit_msgs::RobotTrajectory& traj)
{	
	double score = 0;
	double sum_of_velocities = 0.0;
	// for ( auto& traj_point :traj)
	// {
	// 	// sum_of_velocities = std::inner_product( v1.begin(), v1.end(), v1.begin(), 0 );
	// }
	return score;
}