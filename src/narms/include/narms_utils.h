#ifndef __narms_utils__
#define __narms_utils__
#include <random>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>

namespace
{
	std::mt19937 mt(1729);
}

double sample_from_gaussian(double mean, double variance);

void printPose(geometry_msgs::Pose p);

tf::Transform geoPose2Transform(geometry_msgs::Pose p);

geometry_msgs::Pose getPoseFromRosParamRPYDegrees(std::string prefix);

geometry_msgs::Pose Affine2Pose(Eigen::Affine3d A);

geometry_msgs::Pose convertPoseViaTransform( 
	const geometry_msgs::Pose& pose, 
const tf::Transform& tf);

geometry_msgs::Pose generatePose(double x, double y, double z, double roll, double pitch, double yaw);

geometry_msgs::Pose generateRandomPose();


 void getIKServerRequest(geometry_msgs::Pose pose, std::string planning_group, moveit_msgs::GetPositionIK& ik_srv);

 void getGraspingPoses(const tf::Transform& tf_object, geometry_msgs::Pose& pr2_pose, geometry_msgs::Pose& roman_pose);

 moveit_msgs::CollisionObject createCollisionBox(
	geometry_msgs::Pose object_pose,
	std::string reference_name,
	std::string tf_frame, 
	double sx, double sy, double sz);

 void visualizeObjectPose(geometry_msgs::Pose pose, std::string prefix);

geometry_msgs::Pose sampleHandoff(double c_x1,double c_y1,double c_z1,
	double c_x2, double c_y2, double c_z2, double variance_inflation);
#endif