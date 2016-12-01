#include <vector>

#include <ros/ros.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/SolidPrimitive.h>

#include "narms_utils.h"

/*
moveit_msgs::CollisionObject createCollisionObjectFromMesh(
	std::string mesh_file,
	geometry_msgs::Pose object_pose,
	std::string reference_name,
	std::string tf_frame)
{
	moveit_msgs::CollisionObject collision_object;
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  shapes::Mesh* shape = shapes::createMeshFromResource(
  	mesh_file);
  shapes::constructMsgFromShape(shape, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  collision_object.header.frame_id = tf_frame;
  collision_object.id = reference_name;
  collision_object.meshes.push_back(mesh);
  collision_object.mesh_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}
*/


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "planning_scene_manager");
	moveit::planning_interface::PlanningSceneInterface PSI;

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





	ros::spin();
	return 0;
}