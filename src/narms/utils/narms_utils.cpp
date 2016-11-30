#include "narms_utils.h"

#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <sensor_msgs/JointState.h>

double sample_from_gaussian(double mean, double variance)
	{
		std::normal_distribution<double> gauss(mean,variance);
		return gauss(mt);
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
tf::Transform geoPose2Transform(geometry_msgs::Pose p)
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

	return outPose;
}

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


 void getIKServerRequest(geometry_msgs::Pose pose, std::string planning_group, moveit_msgs::GetPositionIK& ik_srv)
{
	sensor_msgs::JointState js;
	js.position = {0,0,0,0,0,0,0};
	js.velocity = {0,0,0,0,0,0,0};
	if(planning_group.compare("pr2_right_arm")==0)
	{
		js.name = {
			"pr2/r_shoulder_pan_joint",
			"pr2/r_shoulder_lift_joint",
			"pr2/r_upper_arm_roll_joint",
			"pr2/r_elbow_flex_joint",
			"pr2/r_forearm_roll_joint",
			"pr2/r_wrist_flex_joint",
			"pr2/r_wrist_roll_joint"
		};
	}else{
		js.name= {
			"roman/limb_right_joint1",
			"roman/limb_right_joint2",
			"roman/limb_right_joint3",
			"roman/limb_right_joint4",
			"roman/limb_right_joint5",
			"roman/limb_right_joint6",
			"roman/limb_right_joint7"
		};
	}

	moveit_msgs::PositionIKRequest ik_request;
	ik_request.group_name = planning_group;
	ik_request.avoid_collisions = true; // <-- if true, should not need to call validity checker
	ik_request.robot_state.joint_state = js;
	ik_request.attempts = 10;
	ik_request.pose_stamped.header.frame_id = std::string("world");
	ik_request.pose_stamped.pose = pose;
	ik_request.pose_stamped.header.stamp = ros::Time::now();
	ik_srv.request.ik_request = ik_request;
	
}

void getGraspingPoses(geometry_msgs::Pose& input, geometry_msgs::Pose& pr2_pose, geometry_msgs::Pose& roman_pose)
{
	
}