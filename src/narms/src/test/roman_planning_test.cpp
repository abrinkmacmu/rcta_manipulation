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
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

// system includes
#include <actionlib/client/simple_action_client.h>

// project includes
#include <rcta/MoveArmAction.h>

geometry_msgs::Pose generatePose(double x, double y, double z, double roll, double pitch, double yaw)
{
	Eigen::AngleAxisd Qroll, Qpitch, Qyaw;
    Qroll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Qpitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    Qyaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond Q;
	Q =  Qroll * Qpitch * Qyaw ;

	geometry_msgs::Pose p;
	p.position.x = x;
	p.position.y = y;
	p.position.z = z;
	p.orientation.x = Q.x();
	p.orientation.y = Q.y();
	p.orientation.z = Q.z();
	p.orientation.w = Q.w();
	return p;
}

geometry_msgs::Pose generateRandomPose()
{
	double x = 1.0*(std::rand() / double(RAND_MAX));
	double y = 1.0*(std::rand() / double(RAND_MAX)) + 0.5;
	double z = 1.5*(std::rand() / double(RAND_MAX));
	double roll = 3.14*(std::rand() / double(RAND_MAX) -0.5);
	double pitch = 3.14*(std::rand() / double(RAND_MAX) -0.5);
	double yaw = 3.14*(std::rand() / double(RAND_MAX) -0.5);
	return generatePose(x, y, z, roll, pitch, yaw);
}

void printPose(geometry_msgs::Pose p)
{
	std::cout << "pos(" << p.position.x <<
				 ", "<< p.position.y <<
				 ", "<< p.position.z <<
				 ")   q[" << p.orientation.x <<
				 ", "<< p.orientation.y <<
				 ", "<< p.orientation.z <<
				 ", "<< p.orientation.w << "]\n";

}

/** 
 *  @brief geometry_msgs::Pose -> tf::Transform
 */
inline tf::Transform geoPose2Transform(geometry_msgs::Pose p)
{
    tf::Transform tf1;

    tf1.setOrigin( tf::Vector3(
        p.position.x, 
        p.position.y, 
        p.position.z) );

    tf::Quaternion q(
        p.orientation.x, 
        p.orientation.y, 
        p.orientation.z, 
        p.orientation.w); 

    tf1.setRotation(q);

    return tf1;
}

geometry_msgs::Pose getRomanPoseOffset( const geometry_msgs::Pose& pose, 
	const tf::StampedTransform& tf)
{
	geometry_msgs::Pose outPose;

	Eigen::Affine3d A_offset;
	tf::transformTFToEigen(tf, A_offset);

	tf::Transform inputTf;
	Eigen::Affine3d A_pose;
	inputTf = geoPose2Transform(pose);
	tf::transformTFToEigen(inputTf, A_pose);

	Eigen::Affine3d A_out;

	/*
	Pose Math source: 
	http://tinyurl.com/h63ubdb
	*/
	A_out =  A_offset.inverse()* A_pose * A_offset;

	outPose.position.x = A_out.translation()[0];
	outPose.position.y = A_out.translation()[1];
	outPose.position.z = A_out.translation()[2];

	Eigen::Quaterniond Q(A_out.linear());
	outPose.orientation.x = Q.x();
	outPose.orientation.y = Q.y();
	outPose.orientation.z = Q.z();
	outPose.orientation.w = Q.w();

	//std::cout << "A_offset: \n" << A_offset.matrix() << "\n";
	//std::cout << "A_pose: \n" << A_pose.matrix() << "\n";
	//std::cout << "A_out: \n" << A_out.matrix() << "\n";


	//std::cout << "Input pose: \n";
	//printPose(pose);
	//std::cout << "Output pose: \n";
	//printPose(outPose);
	return outPose;

}

void result_callback(
    const actionlib::SimpleClientGoalState& state,
    const rcta::MoveArmResult::ConstPtr& result)
{
    if(result->success){
        ROS_INFO("Planing was Successful!");
    } else {
        ROS_WARN("...");
    }
}


int main(int argc, char*argv[]){
	
	ros::init(argc, argv, "planning_test");
	ros::NodeHandle nh;

	tf::TransformBroadcaster br;
	tf::TransformListener listener;

	tf::StampedTransform roman_tf;
	ros::Duration(1.0).sleep(); // wait for tf listener to start up
	listener.lookupTransform("world","map", ros::Time(0),roman_tf); 


	ros::AsyncSpinner spinner(2);
	spinner.start();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "ik_goals", 0 );
	
	// Roman Specific Messages
	std::cout << "Creating ROMAN movegroup interface\n";
	moveit::planning_interface::MoveGroup::Options opts("right_arm");
	opts.robot_description_ = "roman/robot_description";
	moveit::planning_interface::MoveGroup roman_move_group(opts);
	//std::cout << "Reference frame was: " <<roman_move_group.getPoseReferenceFrame() << "\n";
	

	// For whatever reason movegroup.execute does not move arm
	// Try to use actionlib interface instead
	actionlib::SimpleActionClient<rcta::MoveArmAction> *move_arm_client_;

	moveit::planning_interface::MoveGroup::Plan roman_plan;
	sensor_msgs::JointState roman_js;
	moveit_msgs::PositionIKRequest roman_ik_request;
	moveit_msgs::GetPositionIK roman_ik_srv;
	ros::ServiceClient roman_compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");


	// Build ik service message header for ROMAN****************************
	roman_js.name={"limb_right_joint1",
				   "limb_right_joint2",
				   "limb_right_joint3",
				   "limb_right_joint4",
				   "limb_right_joint5",
				   "limb_right_joint6",
				   "limb_right_joint7"};
	roman_js.position = {0,0,0,0,0,0,0};
	roman_js.velocity = {0,0,0,0,0,0,0};

	roman_ik_request.group_name = std::string("right_arm");
	roman_ik_request.avoid_collisions = true; // <-- if true, should not need to call validity checker
	roman_ik_request.pose_stamped.header.frame_id = std::string("world");
	roman_ik_request.robot_state.joint_state = roman_js;
	roman_ik_request.attempts = 10;


	// Add a visualization marker for shits and giggles**************************
	visualization_msgs::Marker viz_marker;
	viz_marker.header.frame_id = "world";
	viz_marker.ns = "roman";
	viz_marker.id = 0;
	viz_marker.type = visualization_msgs::Marker::CUBE;
	viz_marker.action = visualization_msgs::Marker::ADD;
	viz_marker.scale.x = .05;
	viz_marker.scale.y = .05;
	viz_marker.scale.z = .05;
	viz_marker.color.a = 1.0;
	viz_marker.color.b = 1.0;

	for (int i = 0; i < 100; i++)
	{
		std::cout << i << "\n";

		geometry_msgs::Pose pose = generateRandomPose();
		
		geometry_msgs::Pose roman_planning_pose = getRomanPoseOffset(pose, roman_tf);
		

		//tf::Transform tf1;
	    //tf1 = geoPose2Transform(pose);
	    //br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "world", "pose"));

	    tf::Transform tf2;
	    tf2 = geoPose2Transform(roman_planning_pose);
		br.sendTransform(tf::StampedTransform(tf2, ros::Time::now(), "map", "pose_map")); 
		
		// vizualize pose
		viz_marker.pose = pose;
		viz_marker.header.stamp = ros::Time::now();
		ik_vis_pub.publish(viz_marker);

		ros::Duration(.5).sleep();

		roman_js.header.stamp = ros::Time::now();
		roman_ik_request.pose_stamped.pose = pose;
		roman_ik_request.pose_stamped.header.stamp = ros::Time::now();
		roman_ik_srv.request.ik_request = roman_ik_request;
		if(roman_compute_ik.call(roman_ik_srv))
		{
			//std::cout << "Error code: " << pr2_ik_srv.response.error_code.val << "\n";
			if(roman_ik_srv.response.error_code.val == 1)
			{
				std::cout << "ROMAN can reach goal!\n";

				/* 
				I'm segfaulting :c
				rcta::MoveArmGoal move_arm_goal;
			    move_arm_goal.type = rcta::MoveArmGoal::EndEffectorGoal;
			    move_arm_goal.goal_joint_state.name = roman_js.name;
			    //include the attached object in the goal
			    move_arm_goal.execute_path = true;
			    move_arm_goal.goal_pose = roman_planning_pose;
			    std::cout << "Good here\n";
			    auto result_cb = boost::bind(result_callback, _1, _2);
			    std::cout << "...aaand here\n";
		        move_arm_client_->sendGoal(move_arm_goal, result_cb);
		        std::cout << "...Last place\n";
		        if (move_arm_client_->waitForResult()) {
		            std::cout << "Success!\n";
		        }else{
		        	std::cout << "Failed :c\n";
		        }
				*/

				// try to plan there now
				
				roman_move_group.setPlannerId("arastar");
				roman_move_group.setWorkspace(-2,-2,-2,2,2,2);
				roman_move_group.setPlanningTime(15); // sec
				roman_move_group.setStartState(*roman_move_group.getCurrentState() );
				roman_move_group.setPoseTarget(roman_planning_pose);
				//roman_move_group.setPoseReferenceFrame("world");
				bool suc = roman_move_group.plan(roman_plan);
				if(suc)
				{
					std::cout << "Roman plan found, executing plan with " 
						<< roman_plan.trajectory_.joint_trajectory.points.size() << "\n";
					bool exeSuc = roman_move_group.execute(roman_plan);
					std::cout << "Execution completed with success: " << exeSuc << "\n";
				}
				
			}
		}else{
			std::cout << "Could not call ROMAN IK service\n";
		}



		


	}



	return 0;
}