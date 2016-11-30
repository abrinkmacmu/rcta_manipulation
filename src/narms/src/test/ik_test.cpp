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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>


#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <random>

std::mt19937 mt(1729);

double sample_from_gaussian(double mean, double variance)
	{
		std::normal_distribution<double> gauss(mean,variance);
		return gauss(mt);
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




geometry_msgs::Pose sampleHandoff(double c_x1,double c_y1,double c_z1,
	double c_x2, double c_y2, double c_z2, double variance_inflation)
{
	
	double robo1_reach_m =1.5;
	double robo2_reach_m =1.5;

	double x_center_m = 0.5*(c_x1 + c_x2);
	double y_center_m = 0.5*(c_y1 + c_y2);
	double z_center_m = 0.5*(c_z1 + c_z2);


	// y_center_m = (std::pow(base_distance_m,2)-std::pow(robo2_reach_m,2)+std::pow(robo1_reach_m,2)*0.5)/base_distance_m;

	// double sampling_variance = (4*std::pow(base_distance_m,2)*std::pow(robo1_reach_m)- 
	// 	std::pow(std::pow(base_distance_m,2) 
	// 		- std::pow(robo2_reach_m,2) 
	// 		+std::pow(robo1_reach_m,2),2))/(4*std::pow(base_distance_m,2));

	// //Variance is the radius of the circle divided by three
	// sampling_variance=sampling_variance/3.0;

	double sampling_variance = std::sqrt(std::pow(c_x1-c_x2,2)+std::pow(c_y1-c_y2,2)+std::pow(c_z1-c_z2,2))/3.0;

	//Variance inflation is a number between [0,1], signifying how percentage inflated the sampling should be
	sampling_variance = sampling_variance * (1+variance_inflation);

	std::cout<<"\nSampling Variance :"<<sampling_variance<<std::endl;

	x_center_m = sample_from_gaussian(x_center_m,sampling_variance);
	y_center_m = sample_from_gaussian(y_center_m,sampling_variance);
	z_center_m = sample_from_gaussian(z_center_m,sampling_variance);


	double roll = 3.14*(std::rand() / double(RAND_MAX) -0.5);
	double pitch = 3.14*(std::rand() / double(RAND_MAX) -0.5);
	double yaw = 3.14*(std::rand() / double(RAND_MAX) -0.5);

	//return the generated pose
	return generatePose(x_center_m, y_center_m, z_center_m, yaw, pitch,roll);
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

void transformPose( geometry_msgs::Pose& pose, 
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

	A_out =  A_offset * A_pose;

	pose.position.x = A_out.translation()[0];
	pose.position.y = A_out.translation()[1];
	pose.position.z = A_out.translation()[2];

	Eigen::Quaterniond Q(A_out.linear());
	pose.orientation.x = Q.x();
	pose.orientation.y = Q.y();
	pose.orientation.z = Q.z();
	pose.orientation.w = Q.w();

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



int main(int argc, char*argv[])
{
	ros::init(argc, argv, "ik_test");
	ros::NodeHandle nh;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	ros::Publisher ik_vis_pub = nh.advertise<visualization_msgs::Marker>( "ik_goals", 0 );

	// PR2 Specific Messages
	std::string pr2_planning_group("pr2_right_arm");
	moveit_msgs::GetPositionIK ik_srv;
	ros::ServiceClient compute_ik = nh.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
	
	
	// Roman Specific Messages
	std::string roman_planning_group("roman_right_arm");


	// Add a visualization marker for shits and giggles**************************
	visualization_msgs::Marker viz_marker;
	viz_marker.header.frame_id = "world";
	viz_marker.id = 0;
	viz_marker.type = visualization_msgs::Marker::CUBE;
	viz_marker.action = visualization_msgs::Marker::ADD;
	viz_marker.scale.x = .05;
	viz_marker.scale.y = .05;
	viz_marker.scale.z = .05;
	viz_marker.color.a = 1.0;
	viz_marker.color.r = 1.0;

	bool handoff_sucess = false;
	double sampling_variance_inflation_m = 0.0;
	double sampling_variance_inflation_step_m = 0.01;

	for(int i = 0; i < 100; i++)
	{
		std::cout<<"\nSampled point" << i<< " \n";
		geometry_msgs::Pose randPose = generateRandomPose();

		getIKServerRequest(randPose, pr2_planning_group, ik_srv);

		if(compute_ik.call(ik_srv))
		{
			//std::cout << "Error code: " << pr2_ik_srv.response.error_code.val << "\n";
			if(ik_srv.response.error_code.val == 1){
				std::cout << "PR2 can reach goal!\n";
			}

		}else{
			std::cout << "Could not call PR2 IK service\n";
		}

		// Roman

		getIKServerRequest(randPose, roman_planning_group, ik_srv);

		if(compute_ik.call(ik_srv)){
			
			if(ik_srv.response.error_code.val == 1){
				std::cout << "ROMAN can reach goal!\n";
			}

		}	else {
		
			std::cout << "Could not call ROMAN IK service\n";
		
		}


		// vizualize pose
		viz_marker.pose = randPose;
		viz_marker.header.stamp = ros::Time::now();
		ik_vis_pub.publish(viz_marker);

		ros::Duration(.3).sleep();
	}

return 0;
}
