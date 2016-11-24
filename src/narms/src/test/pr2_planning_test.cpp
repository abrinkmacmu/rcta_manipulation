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

geometry_msgs::Pose convertPoseViaTransform( const geometry_msgs::Pose& pose, 
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


int main(int argc, char*argv[]){
	ros::init(argc, argv, "planning_test");
	ros::NodeHandle nh;

	tf::TransformBroadcaster br;
	tf::TransformListener listener;

	tf::StampedTransform pr2_tf;
	ros::Duration(1.0).sleep(); // wait for tf listener to start up
	listener.lookupTransform("pr2/r_shoulder_pan_link","world", ros::Time(0),pr2_tf);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "ik_goals", 0 );

	// PR2 Specific Messages
	std::cout << "Creating PR2 movegroup interface\n";
	moveit::planning_interface::MoveGroup::Options opts("right_arm");
	opts.robot_description_ = "robot_description";
	moveit::planning_interface::MoveGroup pr2_move_group(opts);
	moveit::planning_interface::MoveGroup::Plan pr2_plan;
	sensor_msgs::JointState pr2_js;
	moveit_msgs::PositionIKRequest pr2_ik_request;
	moveit_msgs::GetPositionIK pr2_ik_srv;
	ros::ServiceClient pr2_compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");


	// build ik service messages for PR2***************************
	pr2_js.name = {"r_shoulder_pan_joint",
			   "r_shoulder_lift_joint",
			   "r_upper_arm_roll_joint",
			   "r_elbow_flex_joint",
			   "r_forearm_roll_joint",
			   "r_wrist_flex_joint",
			   "r_wrist_roll_joint"};	
	pr2_js.position = {0,0,0,0,0,0,0};
	pr2_js.velocity = {0,0,0,0,0,0,0};

	pr2_ik_request.group_name = std::string("right_arm");
	pr2_ik_request.avoid_collisions = true; // <-- if true, should not need to call validity checker
	//pr2_ik_request.ik_link_name = "r_wrist_roll_link";
	pr2_ik_request.pose_stamped.header.frame_id = std::string("world");
	pr2_ik_request.robot_state.joint_state = pr2_js;
	pr2_ik_request.attempts = 10;


	// Add a visualization marker for shits and giggles**************************
	visualization_msgs::Marker viz_marker;
	viz_marker.header.frame_id = "world";
	viz_marker.ns = "pr2";
	viz_marker.id = 0;
	viz_marker.type = visualization_msgs::Marker::CUBE;
	viz_marker.action = visualization_msgs::Marker::ADD;
	viz_marker.scale.x = .05;
	viz_marker.scale.y = .05;
	viz_marker.scale.z = .05;
	viz_marker.color.a = 1.0;
	viz_marker.color.r = 1.0;

	for (int i = 0; i < 100; i++)
	{
		std::cout << i << "\n";

		geometry_msgs::Pose pose = generateRandomPose();

		//geometry_msgs::Pose pr2_planning_pose = convertPoseViaTransform(pose, pr2_tf);
		

		tf::Transform tf1;
	    tf1 = geoPose2Transform(pose);
	    br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "world", "pose"));

	    //tf::Transform tf2;
	    //tf2 = geoPose2Transform(pr2_planning_pose);
		//br.sendTransform(tf::StampedTransform(tf2, ros::Time::now(), "pr2/r_shoulder_pan_link", "pose_map"));

		// vizualize pose
		viz_marker.pose = pose;
		viz_marker.header.stamp = ros::Time::now();
		ik_vis_pub.publish(viz_marker);

		ros::Duration(.3).sleep();
		
		

		pr2_js.header.stamp = ros::Time::now();
		pr2_ik_request.pose_stamped.pose = pose;
		pr2_ik_request.pose_stamped.header.stamp = ros::Time::now();
		pr2_ik_srv.request.ik_request = pr2_ik_request;
		if(pr2_compute_ik.call(pr2_ik_srv))
		{
			//std::cout << "Error code: " << pr2_ik_srv.response.error_code.val << "\n";
			if(pr2_ik_srv.response.error_code.val == 1)
			{
				std::cout << "PR2 can reach goal!\n";
				// try to plan now
				pr2_move_group.setStartState(*pr2_move_group.getCurrentState() );
				pr2_move_group.setPlannerId("RRTkConfigDefault");
				pr2_move_group.setPlanningTime(10); // sec
				//pr2_move_group.setPoseReferenceFrame("world");
				pr2_move_group.setPoseTarget(pose);
				//pr2_move_group.setPositionTarget(pose.position.x, pose.position.y, pose.position.z);
				bool suc = pr2_move_group.plan(pr2_plan);
				if(suc)
				{
					std::cout  << "PR2 plan found, executing now...\n";
					pr2_move_group.execute(pr2_plan);
				}
			}
		}else{
			std::cout << "Could not call PR2 IK service\n";
		}


		


	}



	return 0;
}