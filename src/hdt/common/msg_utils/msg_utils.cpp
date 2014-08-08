#include "msg_utils.h"

#include <Eigen/Dense>
#include <algorithm>

namespace msg_utils
{

bool contains_joints(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joints)
{
    for (const std::string& joint_name : joints) {
        if (std::find(joint_state.name.begin(), joint_state.name.end(), joint_name) == joint_state.name.end()) {
            return false;
        }
    }
    return true;
}

bool contains_only_joints(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joints)
{
    return vector_sets_equal(joint_state.name, joints);
}

bool reorder_joints(sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_order)
{
    if (!contains_only_joints(joint_state, joint_order)) {
        return false;
    }

    for (std::size_t i = 0; i < joint_order.size(); ++i) {
        const std::string& jo_joint_name = joint_order[i];
        for (std::size_t j = i; j < joint_state.name.size(); ++j) {
            std::string& js_joint_name = joint_state.name[j];
            if (js_joint_name == jo_joint_name && i != j) {
                swap(joint_state.name[i], joint_state.name[j]);
                std::swap(joint_state.position[i], joint_state.position[j]);
                std::swap(joint_state.velocity[i], joint_state.velocity[j]);
                std::swap(joint_state.effort[i], joint_state.effort[j]);
            }
        }
    }

    return true;
}

bool reorder_joints(trajectory_msgs::JointTrajectory& joint_trajectory, const std::vector<std::string>& joint_order)
{
    if (!vector_sets_equal(joint_trajectory.joint_names, joint_order)) {
        return false;
    }

    for (std::size_t i = 0; i < joint_order.size(); ++i) {
        const std::string& jo_joint_name = joint_order[i];
        for (std::size_t j = i; j < joint_trajectory.joint_names.size(); ++j) {
            std::string& js_joint_name = joint_trajectory.joint_names[j];
            if (js_joint_name == jo_joint_name && i != j) {
                swap(joint_trajectory.joint_names[i], joint_trajectory.joint_names[j]);
                for (trajectory_msgs::JointTrajectoryPoint point : joint_trajectory.points) {
                    if (!point.positions.empty()) {
                        std::swap(point.positions[i], point.positions[j]);
                    }
                    if (!point.velocities.empty()) {
                        std::swap(point.velocities[i], point.velocities[j]);
                    }
                    if (!point.accelerations.empty()) {
                        std::swap(point.accelerations[i], point.accelerations[j]);
                    }
                }
            }
        }
    }

    return true;
}

visualization_msgs::Marker create_arrow_marker(const geometry_msgs::Vector3 &scale)
{
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.seq = 0;
    arrow_marker.header.stamp = ros::Time(0);
    arrow_marker.header.frame_id = "";
    arrow_marker.ns = "";
    arrow_marker.id = 0;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.position.x = 0.0;
    arrow_marker.pose.position.y = 0.0;
    arrow_marker.pose.position.z = 0.0;
    arrow_marker.pose.orientation.w = 1.0;
    arrow_marker.pose.orientation.x = 0.0;
    arrow_marker.pose.orientation.y = 0.0;
    arrow_marker.pose.orientation.z = 0.0;
    arrow_marker.scale.x = scale.x;
    arrow_marker.scale.y = scale.y;
    arrow_marker.scale.z = scale.z;
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 0.7;
    arrow_marker.lifetime = ros::Duration(0);
    arrow_marker.frame_locked = false;
    return arrow_marker;
}

visualization_msgs::MarkerArray create_triad_marker_arr(const geometry_msgs::Vector3& scale)
{
    visualization_msgs::MarkerArray markers;

    // create an arrow marker for the x-axis
    visualization_msgs::Marker m = create_arrow_marker(scale);
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.7;
    m.id = 0;
    markers.markers.push_back(m);

    // create an arrow marker for the y-axis
    Eigen::AngleAxisd rotate_z(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    Eigen::Quaterniond q = rotate_z * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.id = 1;
    markers.markers.push_back(m);

    // create an arrow marker for the z-axis
    Eigen::AngleAxisd rotate_y(-M_PI / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0));
    q = rotate_y * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.id = 2;
    markers.markers.push_back(m);

    visualization_msgs::Marker origin_marker;
    origin_marker.header.seq = 0;
    origin_marker.header.stamp = ros::Time(0);
    origin_marker.header.frame_id = "";
    origin_marker.ns = "";
    origin_marker.id = 0;
    origin_marker.type = visualization_msgs::Marker::CUBE;
    origin_marker.action = visualization_msgs::Marker::ADD;
    origin_marker.pose.position.x = origin_marker.pose.position.y = origin_marker.pose.position.z = 0.0;
    origin_marker.pose.orientation.w = 1.0;
    origin_marker.pose.orientation.x = origin_marker.pose.orientation.y = origin_marker.pose.orientation.z = 0.0;
    origin_marker.scale.x = origin_marker.scale.y = origin_marker.scale.z = scale.y; // note: assume that scale.y and scale.z are the same
    origin_marker.color.r = origin_marker.color.g = origin_marker.color.b = 0.9;
    origin_marker.color.a = 1.0;
    origin_marker.lifetime = ros::Duration(0);
    origin_marker.frame_locked = false;
    origin_marker.id = 3;
    markers.markers.push_back(origin_marker);

    return markers;
}

} // namespace msg_utils
