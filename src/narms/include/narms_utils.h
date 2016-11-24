#ifndef __narms_utils__
#define __narms_utils__
#include <random>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>


namespace
{
	std::mt19937 mt(1729);
}

double sample_from_gaussian(double mean, double variance);

void printPose(geometry_msgs::Pose p);

tf::Transform geoPose2Transform(geometry_msgs::Pose p);

geometry_msgs::Pose convertPoseViaTransform( 
	const geometry_msgs::Pose& pose, 
	const tf::StampedTransform& tf);

#endif