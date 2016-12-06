
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
				geometry_msgs::Pose startBasePose, geometry_msgs::Pose goalBasePose, 
	std::string startRobot, std::string goalRobot, geometry_msgs::Pose& handoffPose,
	unsigned int num_of_handoff_samples,
	ros::ServiceClient& compute_ik,
	ros::ServiceClient& robot1_move_arm_server,
	ros::ServiceClient& robot2_move_arm_server,
	double planning_time)
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
	// std::vector<moveit_msgs::RobotTrajectory> robot1_trajectories;
	// std::vector<moveit_msgs::RobotTrajectory> robot2_trajectories;

	tf::TransformBroadcaster tf_broadcaster;

	double sampling_variance_inflation_m = 0.02;

	// Best Score till now for a trajectory
	double best_score = std::numeric_limits<double>::max();
	combinedTrajectories best_trajectory_pair;


	narms::target_pose robot1_mas_request;
	narms::target_pose robot2_mas_request;

	while(handoffs_found!=num_of_handoff_samples)
	{
		//continue sampling till you find a solution reachable by both arms
		if(!ros::ok()) {return 0;}

		// geometry_msgs::Pose handoffPose = generateRandomPose(); // Replace with sampled pose

		geometry_msgs::Pose current_handoff_pose =sampleHandoff(startBasePose.position.x,startBasePose.position.y,startBasePose.position.z,
								goalBasePose.position.x,goalBasePose.position.y,goalBasePose.position.z,sampling_variance_inflation_m);
 								
		tf::StampedTransform handoffTf(geoPose2Transform(current_handoff_pose), ros::Time::now(), "world_link", "object");

		//Get the grasping poses for each robot
		geometry_msgs::Pose robot1GraspPose;
		geometry_msgs::Pose robot2GraspPose;

		if (startRobot == "roman")
		{
			getGraspingPoses(handoffTf, robot2GraspPose, robot1GraspPose);
			getIKServerRequest(robot1GraspPose, "roman_right_arm", ik_srv_robot1);
			getIKServerRequest(robot2GraspPose, "pr2_right_arm", ik_srv_robot2);
			
		}
		else
		{
			getGraspingPoses(handoffTf, robot1GraspPose, robot2GraspPose);
			getIKServerRequest(robot1GraspPose, "pr2_right_arm", ik_srv_robot1);
			getIKServerRequest(robot2GraspPose, "roman_right_arm", ik_srv_robot2);

		}
		

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
		// std::cout << "Robot1 compute_IK call time: " << (t1-t0).toSec() << "\n";

		if(ik_suc){
			if(ik_srv_robot1.response.error_code.val == 1){
				//std::cout << "Robot1 can reach goal!\n";
				n_reached++;
			}
			
			// else
			// {
			// 	std::cout<< "FAILED!\n";
			// }
			
		}else{
			std::cout << "Could not call Robot1 IK service\n";
		}

		// Roman IK **********************************************************************************

		// getIKServerRequest(goalPose, goalRobot, ik_srv);

		t0 = ros::WallTime::now();
		ik_suc = compute_ik.call(ik_srv_robot2);
		t1 = ros::WallTime::now();
		// std::cout << "Robot2 compute_IK call time: " << (t1-t0).toSec() << "\n";


		if(ik_suc){
			if(ik_srv_robot2.response.error_code.val == 1){
				//std::cout << "Robot2 can reach goal!\n";
				n_reached++;
			}
			// else
			// {
			// 	std::cout<< "FAILED!\n";
			// }
		}	else {
			 std::cout << "Could not call Robot2 IK service\n";
		}

		// Planning and Execution ******************************************************************

		if (n_reached >= 2)
		{
		// Solution is feasible for both PR2 and ROMAN, 
		// Get the trajectory for this point and evaluate later
		
		ROS_INFO("SAMPLING POINT FOUND..FINDING PLANS");		
		int n_planned = 0;

		// PR2 planning and execution
		
		robot1_mas_request.request.pose = robot1GraspPose;
		robot1_mas_request.request.execute_plan=false;
		robot1_mas_request.request.planning_time=planning_time;


		ros::WallTime t2 = ros::WallTime::now();
		robot1_move_arm_server.call(robot1_mas_request);
		ros::WallTime t3 = ros::WallTime::now();
		std::cout << "pr2 move_arm_server call time: " << (t3-t2).toSec() << "\n";

		if(robot1_mas_request.response.result == true){
			//std::cout << "PR2 Trajectory Found!\n";
			n_planned ++;
		}
		else
		{
			//std::cout << "PR2 Trajectory Not Found!\n";
		}

		// Roman planning and execution
		// narms::target_pose robot2_mas_request;
		robot2_mas_request.request.pose = robot2GraspPose;
		robot2_mas_request.request.execute_plan=false;
		robot2_mas_request.request.planning_time=planning_time;

		t2 = ros::WallTime::now();
		robot2_move_arm_server.call(robot2_mas_request);
		t3 = ros::WallTime::now();
		std::cout << "roman move_arm_server call time: " << (t3-t2).toSec() << "\n";


		if(robot2_mas_request.response.result == true)
		{
			//std::cout << "ROMAN Trajectory Found!\n";
			n_planned++;
		}
		else
			{
				//std::cout << "ROMAN Trajectory Not Found!\n";
			}

		if(n_planned >= 2)
			{
				//std::cout << "\nBOTH ARM REACHED OBJECT\n";
				printPose(handoffPose);
				//ros::Duration(2).sleep();
				// robot1_trajectories.push_back(robot1_mas_request.response.traj);
				// robot2_trajectories.push_back(robot2_mas_request.response.traj.joint_trajectory.points);
				++handoffs_found;
				std::cout<<"\nhandoffs_found: "<<handoffs_found;
				double score1=score_trajectory(robot1_mas_request.response.traj.joint_trajectory.points);
				double score2=score_trajectory(robot2_mas_request.response.traj.joint_trajectory.points);

				std::cout<<"\nScores: "<<score1<<" "<<score2;

				if (score1+score2 < best_score)
				{
					best_trajectory_pair.robot1_grasp = robot1GraspPose;
					best_trajectory_pair.robot2_grasp = robot2GraspPose;
					handoffPose = current_handoff_pose;
					best_score = score1+score2;

				}
			}
		}
	}

	//Execute the best Trajectory

	// ROS_INFO("Executing Best Trajectories");
	// robot1_mas_request.request.pose = best_trajectory_pair.robot1_grasp;
	// robot1_mas_request.request.execute_plan=true;
	// robot1_move_arm_server.call(robot1_mas_request);

	// robot2_mas_request.request.pose = best_trajectory_pair.robot2_grasp;
	// robot2_mas_request.request.execute_plan=true;
	// robot2_move_arm_server.call(robot2_mas_request);

	std::cout<<"\nFinal handoffPose: "<<handoffPose;
	std::cout << "\nBest score: " << best_score;
	
	return true;
}


// double score_trajectory(std::vector<trajectory_msgs::JointTrajectoryPoint>& traj)
// {	
// 	double score = 0;
// 	double sum_of_velocities = 0.0;
// 	double sum_of_accelerations = 0.0;
// 	for ( auto& traj_point :traj)
// 	{	
// 		// std::cout<<traj_point<<std::endl;

// 		auto joint_velocities = traj_point.velocities;
// 		auto joint_accelerations = traj_point.accelerations;

// 		sum_of_velocities+= std::inner_product( joint_velocities.begin(), joint_velocities.end(), joint_velocities.begin(), 0.0 );
// 		sum_of_accelerations+= std::inner_product( joint_accelerations.begin(), joint_accelerations.end(), joint_accelerations.begin(), 0.0 );
		
// 		// std::cout<<traj_point;
// 		// std::cout<<"\npoint scores = "<<sum_of_velocities<<" "<<sum_of_accelerations;

// 	}

// 	std::cout<<"\nSum of velocities: "<<sum_of_velocities<<" , Sum of accelerations: "<<sum_of_accelerations;
// 	score = sum_of_accelerations + sum_of_velocities;
// 	return score;
// }



double score_trajectory(std::vector<trajectory_msgs::JointTrajectoryPoint>& traj)
{    
    double score = 0;
    double sum_of_velocities = 0.0;
    double sum_of_accelerations = 0.0;
    
    double discretization = 0.017453292519943;
    for ( size_t idx = 1; idx<traj.size();++idx)
    {    


        auto prev_pt = traj[idx-1];
        auto curr_pt = traj[idx];


        for (size_t joint_idx=0; joint_idx<prev_pt.positions.size();++joint_idx)
        {
            double joint_travel = std::fabs(curr_pt.positions[joint_idx]-prev_pt.positions[joint_idx]);
            joint_travel/=discretization;
            sum_of_velocities+=joint_travel*std::pow(curr_pt.velocities[joint_idx],2);
            sum_of_accelerations+=joint_travel*std::pow(curr_pt.accelerations[joint_idx],2);
        }
        // std::cout<<traj_point<<std::endl;


        // auto joint_velocities = traj_point.velocities;
        // auto joint_accelerations = traj_point.accelerations;


        // sum_of_velocities+= std::inner_product( joint_velocities.begin(), joint_velocities.end(), joint_velocities.begin(), 0.0 );
        // sum_of_accelerations+= std::inner_product( joint_accelerations.begin(), joint_accelerations.end(), joint_accelerations.begin(), 0.0 );
        
        // std::cout<<traj_point;
        std::cout<<"\npoint scores v= "<<sum_of_velocities<<", acc= "<<sum_of_accelerations;


    }


    std::cout<<"\nSum of velocities: "<<sum_of_velocities<<" , Sum of accelerations: "<<sum_of_accelerations;
    score = sum_of_accelerations + sum_of_velocities;
    return score;
}


