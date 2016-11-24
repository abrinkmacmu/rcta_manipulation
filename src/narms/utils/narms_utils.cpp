#include "narms_utils.h"

#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

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