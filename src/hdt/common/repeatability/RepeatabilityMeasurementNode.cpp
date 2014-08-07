#include "RepeatabilityMeasurementNode.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <eigen_conversions/eigen_msg.h>
#include <sbpl_geometry_utils/utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/common/stringifier/stringifier.h>

std::string parseandpad(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    const int max_chars = 150 + 1;
    char buffer[max_chars] = { 0 };
    int retVal = vsnprintf(buffer, max_chars, fmt, args);

    if (retVal >= 0 && retVal < max_chars) {
        for (int r = retVal; r < max_chars - 1; ++r) {
            buffer[r] = ' ';
        }
    }
    buffer[max_chars - 1] = '\0';
    va_end(args);
    return std::string(buffer);
}

#define AU_INFO(fmt, ...) ROS_INFO("%s", parseandpad(fmt, ##__VA_ARGS__).c_str())

namespace hdt
{

std::string to_string(const hdt::JointState& js)
{
    std::stringstream ss;
    ss << "[ ";
    for (const double d : js) {
        ss << d << ' ';
    }
    ss << ']';
    return ss.str();
}

} // namespace hdt

RepeatabilityMeasurementNode::RepeatabilityMeasurementNode() :
    nh_(),
    ph_("~"),
    workspace_min_(),
    workspace_max_(),
    roll_offset_degs_(),
    pitch_offset_degs_(),
    yaw_offset_degs_(),
    num_samples_(),
    sample_res_(),
    num_roll_samples_(),
    num_pitch_samples_(),
    num_yaw_samples_(),
    roll_res_(),
    pitch_res_(),
    yaw_res_(),
    listener_(),
    joint_cmd_pub_(),
    joint_states_sub_(),
    egress_positions_(),
    sample_eef_poses_(),
    camera_frame_(),
    mount_frame_(),
    camera_frame_to_tool_frame_rotation_(),
    last_markers_msg_(),
    last_joint_state_msg_(),
    urdf_description_(),
    robot_model_(),
    pending_command_(false)
{
}

int RepeatabilityMeasurementNode::run()
{
    // overview:
    //     1. Create a number of known safe configurations that we can
    //        reasonably interpolate to and from for most poses in the visible
    //        workspace
    //     2. Create a sparse sampling of visible/reachable poses for the end effector
    //     3. For each sample end effector pose
    //     4.     For each known safe configuration
    //     5.         Add to measure of the variance in the actual measured pose of the end effector at this end effector pose
    //

    // 1. egress positions are read from config
    if (!download_params()) { // errors printed within
        return FAILED_TO_INITIALIZE;
    }

    if (!nh_.getParam("robot_description", urdf_description_)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from param server");
        return FAILED_TO_INITIALIZE;
    }
    if (!(robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_description_))) {
        ROS_ERROR("Failed to load Robot Model from URDF");
        return FAILED_TO_INITIALIZE;
    }

    // initialize simple action client for move arm commands
    move_arm_command_action_name_ = "move_arm_command";
    move_arm_command_client_.reset(new MoveArmCommandActionClient(move_arm_command_action_name_, false));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Action Client");
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate waitLoopRate(1.0);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!move_arm_command_client_->isServerConnected()) {
            AU_INFO("Waiting to connect to action server %s", move_arm_command_action_name_.c_str());
        }
        else {
            break;
        }
        waitLoopRate.sleep();
    }

    AU_INFO("Connected to action server '%s'", move_arm_command_action_name_.c_str());

    const double ninety_rads = M_PI / 2.0;
    camera_frame_to_tool_frame_rotation_ = Eigen::Affine3d(Eigen::AngleAxisd(ninety_rads, Eigen::Vector3d::UnitZ()));

    // subscribe to joint states and ar markers
    joint_cmd_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("command", 1);
    sample_eef_pose_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("sample_pose_markers", 1);
    joint_states_sub_ = nh_.subscribe("joint_states", 1, &RepeatabilityMeasurementNode::joint_states_callback, this);

    // 2. create sparse discretized sampling
    std::vector<geometry_msgs::Pose> sample_poses = generate_sample_poses();

    AU_INFO("Minimum x is %0.3f", std::min_element(sample_poses.begin(), sample_poses.end(),
                    [](const geometry_msgs::Pose& p, const geometry_msgs::Pose& q)
                    { return p.position.x < q.position.x; })->position.x);
    AU_INFO("Minimum y is %0.3f", std::min_element(sample_poses.begin(), sample_poses.end(), [](const geometry_msgs::Pose& p, const geometry_msgs::Pose& q) { return p.position.y < q.position.y; })->position.y);
    AU_INFO("Minimum z is %0.3f", std::min_element(sample_poses.begin(), sample_poses.end(), [](const geometry_msgs::Pose& p, const geometry_msgs::Pose& q) { return p.position.z < q.position.z; })->position.z);

    AU_INFO("Generated %zd sample end effector locations within view of the camera", sample_poses.size());

    // try to move to each ik solution
    for (const auto& pose : sample_poses)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(0);//::now();
        pose_stamped.header.seq = 0;
        pose_stamped.header.frame_id = camera_frame_;
        pose_stamped.pose = pose;
        geometry_msgs::PoseStamped eef_pose_mount_frame;

        if (!listener_.waitForTransform(mount_frame_, camera_frame_, ros::Time::now(), ros::Duration(5.0))) {
            ROS_WARN("Unable to transform sample pose in the camera frame to the mounting frame");
            continue;
        }

        try {
            listener_.transformPose(mount_frame_, pose_stamped, eef_pose_mount_frame);
        }
        catch (const tf::TransformException& ex) {
            ROS_WARN("Failed to transform pose from frame '%s' to '%s'. Fuck TF!", camera_frame_.c_str(), mount_frame_.c_str());
        }

        ROS_DEBUG("Transformed %s (%s -> %s) = %s",
                to_string(pose_stamped.pose).c_str(), pose_stamped.header.frame_id.c_str(),
                eef_pose_mount_frame.header.frame_id.c_str(), to_string(eef_pose_mount_frame.pose).c_str());

        Eigen::Affine3d mount_to_eef;
        tf::poseMsgToEigen(eef_pose_mount_frame.pose, mount_to_eef);

//        AU_INFO("Measuring repeatability for end effector pose %s", to_string(mount_to_eef).c_str());

        auto publish_triad = [&](){
            // publish the new pose marker
            geometry_msgs::Vector3 marker_scale;
            marker_scale.x = 0.1;
            marker_scale.y = marker_scale.z = 0.01;
            visualization_msgs::MarkerArray pose_markers = msg_utils::create_triad_marker_arr(marker_scale);
            for (visualization_msgs::Marker& marker : pose_markers.markers) {
                Eigen::Affine3d marker_pose_eef_frame;
                tf::poseMsgToEigen(marker.pose, marker_pose_eef_frame);
                Eigen::Affine3d mount_to_marker = mount_to_eef * marker_pose_eef_frame;
                tf::poseEigenToMsg(mount_to_marker, marker.pose);
                marker.header.frame_id = mount_frame_;
            }
            sample_eef_pose_marker_pub_.publish(pose_markers);
        };
        publish_triad();

        // move back and forth between the egress position and the ik solution until all transitions have been attempted
        for (const auto& egress_position : egress_positions_) {
            // seed the ik search with the free angle at the egress pose
            std::vector<double> seed(robot_model_->joint_names().size(), 0.0);
            seed[robot_model_->free_angle_index()] = egress_position.a4;

            hdt::IKSolutionGenerator iksols =
                    robot_model_->search_all_ik_solutions(mount_to_eef, seed, sbpl::utils::ToRadians(1.0));
            int num_ik_solutions_attempted = 0;
            std::vector<double> iksol;
            while (iksols(iksol))
            {
                publish_triad();
                AU_INFO("  Moving to egress position %s", to_string(egress_position).c_str());
                if (!move_to_position(egress_position))
                {
                    ROS_WARN("    Failed to move to egress position");
                    continue;
                }

                // move to the ik position
                hdt::JointState js;
                std::memcpy(&js, iksol.data(), iksol.size() * sizeof(double));
                AU_INFO("  Moving to IK solution %s", to_string(js).c_str());
                if (!move_to_position(js))
                {
                    ROS_WARN("    Failed to move to ik solution");
                    continue;
                }

                // record the marker pose and compare with the other times that we have moved to this end effector pose
                AU_INFO("  Recording marker pose at this joint position");
                geometry_msgs::Pose marker_pose_camera_frame;
                if (!track_marker_pose(ros::Duration(1.0), marker_pose_camera_frame)) {
                    ROS_WARN("Unable to detect marker at pose %s", to_string(pose).c_str());
                }

                ++num_ik_solutions_attempted;
            }

            if (num_ik_solutions_attempted != 0)
                AU_INFO("  Attempted %d IK Solutions", num_ik_solutions_attempted);
        }
    }

    return SUCCESS;
}

void RepeatabilityMeasurementNode::ar_markers_callback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
    last_markers_msg_ = msg;
}

void RepeatabilityMeasurementNode::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    last_joint_state_msg_ = msg;
}

bool RepeatabilityMeasurementNode::download_params()
{
    std::vector<double> workspace_min_coords;
    std::vector<double> workspace_max_coords;

    if (!ph_.getParam("workspace_min", workspace_min_coords) ||
        !ph_.getParam("workspace_max", workspace_max_coords))
    {
        ROS_ERROR("Failed to retrieve 'workspace_min' or 'workspace_max' parameters from config");
        return false;
    }

    if (workspace_min_coords.size() != 3 || workspace_max_coords.size() != 3) {
        ROS_ERROR("Invalid workspace coordinates from config");
        return false;
    }

    workspace_min_(0) = workspace_min_coords[0];
    workspace_min_(1) = workspace_min_coords[1];
    workspace_min_(2) = workspace_min_coords[2];

    workspace_max_(0) = workspace_max_coords[0];
    workspace_max_(1) = workspace_max_coords[1];
    workspace_max_(2) = workspace_max_coords[2];

    if (!ph_.getParam("roll_offset_degs", roll_offset_degs_) ||
        !ph_.getParam("pitch_offset_degs", pitch_offset_degs_) ||
        !ph_.getParam("yaw_offset_degs", yaw_offset_degs_))
    {
        ROS_ERROR("Failed to retrieve angle offset parameters from config");
        return false;
    }

    if (!ph_.getParam("sample_resolution_x", sample_res_(0)) ||
        !ph_.getParam("sample_resolution_y", sample_res_(1)) ||
        !ph_.getParam("sample_resolution_z", sample_res_(2)))
    {
        ROS_ERROR("Failed to retrieve sampling parameters");
        return false;
    }

    if (!download_egress_positions()) {
        return false;
    }

    if (!ph_.getParam("camera_frame", camera_frame_)) {
        ROS_ERROR("Failed to retrieve 'camera_frame' from config");
        return false;
    }

    mount_frame_ = "arm_mount_panel_dummy";

    return true;
}

bool RepeatabilityMeasurementNode::download_egress_positions()
{
    XmlRpc::XmlRpcValue egress_positions;
    if (!ph_.getParam("egress_positions", egress_positions) ||
        egress_positions.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }

    // read in each egress position
    for (int i = 0; i < egress_positions.size(); ++i) {
        XmlRpc::XmlRpcValue joint_vector = egress_positions[i];
        // assert the size of the joint vector
        if (joint_vector.size() != 7) {
            ROS_WARN("Joint vector has incorrect size (%d). Expected 7.", joint_vector.size());
            return false;
        }

        // assert the types of the joint element
        for (int i = 0; i < 7; ++i) {
            XmlRpc::XmlRpcValue::Type type = joint_vector[i].getType();
            if (type != XmlRpc::XmlRpcValue::TypeDouble) {
                ROS_WARN("Joint vector element has incorrect type. (%s) Expected double", to_cstring(type));
                return false;
            }
        }

        // create the joint stat
        hdt::JointState joint_state(
            (double)joint_vector[0],
            (double)joint_vector[1],
            (double)joint_vector[2],
            (double)joint_vector[3],
            (double)joint_vector[4],
            (double)joint_vector[5],
            (double)joint_vector[6]);

        egress_positions_.push_back(joint_state);

        AU_INFO("Read in egress position %s", to_string(joint_state).c_str());
    }

    return true;
}

const char* RepeatabilityMeasurementNode::to_cstring(XmlRpc::XmlRpcValue::Type type)
{
    switch (type)
    {
    case XmlRpc::XmlRpcValue::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::TypeDouble:
        return "Double";
    case XmlRpc::XmlRpcValue::TypeInt:
        return "Int";
    case XmlRpc::XmlRpcValue::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::TypeStruct:
        return "Struct";
    }
}

std::vector<geometry_msgs::Pose> RepeatabilityMeasurementNode::generate_sample_poses()
{
    const double sample_resolution_m = 0.10; // ideal linear discretization
    const double sample_angle_res_degs = 30.0; // ideal angular discretization

    Eigen::Vector3d workspace_size = workspace_max_ - workspace_min_;

    num_samples_(0) = (int)std::round(workspace_size(0) / sample_resolution_m) + 1;
    num_samples_(1) = (int)std::round(workspace_size(1) / sample_resolution_m) + 1;
    num_samples_(2) = (int)std::round(workspace_size(2) / sample_resolution_m) + 1;

    sample_res_(0) = workspace_size(0) / (num_samples_(0) - 1);
    sample_res_(1) = workspace_size(1) / (num_samples_(1) - 1);
    sample_res_(2) = workspace_size(2) / (num_samples_(2) - 1);

    num_roll_samples_ = (int)std::round(2 * roll_offset_degs_ / sample_angle_res_degs) + 1;
    roll_res_ = 2 * roll_offset_degs_ / (num_roll_samples_ - 1);

    num_pitch_samples_ = (int)std::round(2 * pitch_offset_degs_ / sample_angle_res_degs) + 1;
    pitch_res_ = 2 * pitch_offset_degs_ / (num_pitch_samples_ - 1);

    num_yaw_samples_ = (int)std::round(2 * yaw_offset_degs_ / sample_angle_res_degs) + 1;
    yaw_res_ = 2 * yaw_offset_degs_ / (num_yaw_samples_ - 1);
    std::vector<geometry_msgs::Pose> sample_poses;

    ROS_INFO("Sampling x locations in [%0.3f, %0.3f] at %0.3f m with %d samples", workspace_min_(0), workspace_max_(0), sample_res_(0), num_samples_(0));
    ROS_INFO("Sampling y locations in [%0.3f, %0.3f] at %0.3f m with %d samples", workspace_min_(1), workspace_max_(1), sample_res_(1), num_samples_(1));
    ROS_INFO("Sampling z locations in [%0.3f, %0.3f] at %0.3f m with %d samples", workspace_min_(2), workspace_max_(2), sample_res_(2), num_samples_(2));

    for (int x = 0; x < num_samples_(0); ++x)
    {
        double sample_x = workspace_min_(0) + x * sample_res_(0);
        for (int y = 0; y < num_samples_(1); ++y)
        {
            double sample_y = workspace_min_(1) + y * sample_res_(1);
            for (int z = 0; z < num_samples_(2); ++z)
            {
                double sample_z = workspace_min_(2) + z * sample_res_(2);
                for (int roll = 0; roll < num_roll_samples_; ++roll)
                {
                    double sample_roll = -roll_offset_degs_ + roll * roll_res_;
                    for (int pitch = 0; pitch < num_pitch_samples_; ++pitch)
                    {
                        double sample_pitch = -pitch_offset_degs_ + pitch * pitch_res_;
                        for (int yaw = 0; yaw < num_yaw_samples_; ++yaw)
                        {
                            double sample_yaw = -yaw_offset_degs_ + yaw * yaw_res_;

                            Eigen::Affine3d sample_transform_camera_frame =
                                Eigen::Translation3d(sample_x, sample_y, sample_z) *
                                Eigen::AngleAxisd(sbpl::utils::ToRadians(sample_roll), Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(sbpl::utils::ToRadians(sample_pitch), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(sbpl::utils::ToRadians(sample_yaw), Eigen::Vector3d::UnitZ()) *
                                camera_frame_to_tool_frame_rotation_;

                            geometry_msgs::Pose sample_pose;
                            tf::poseEigenToMsg(sample_transform_camera_frame, sample_pose);

                            ROS_INFO("Adding sample pose %s", to_string(sample_pose).c_str());
                            sample_poses.push_back(sample_pose);
                        }
                    }
                }
            }
        }
    }

    return sample_poses;
}

bool RepeatabilityMeasurementNode::move_to_position(const hdt::JointState& position)
{
    if (!move_arm_command_client_->isServerConnected()) {
        ROS_WARN("Move Arm Command Client (%s) is not connected", move_arm_command_action_name_.c_str());
        return false;
    }

    std::vector<double> joint_vector =
            { position.a0, position.a1, position.a2, position.a3, position.a4, position.a5, position.a6 };

    hdt::MoveArmCommandGoal goal;

    goal.type = hdt::MoveArmCommandGoal::JointGoal;
    goal.goal_joint_state.header.stamp = ros::Time::now();
    goal.goal_joint_state.header.frame_id = "";
    goal.goal_joint_state.header.seq = 0;
    goal.goal_joint_state.name = robot_model_->joint_names();
    goal.goal_joint_state.position = joint_vector;

    auto result_callback = boost::bind(&RepeatabilityMeasurementNode::move_arm_command_result_cb, this, _1, _2);
    move_arm_command_client_->sendGoal(goal, result_callback);
    pending_command_ = true;
    ros::Rate lr(2);
    while (ros::ok() && pending_command_)
    {
        // TODO: manually check for timeout here
        ros::spinOnce();
        lr.sleep();
    }

    return true;
}

bool RepeatabilityMeasurementNode::track_marker_pose(const ros::Duration& listen_duration, geometry_msgs::Pose& pose)
{
    ros::Time start = ros::Time::now();
    while (ros::ok() && ros::Time::now() < start + listen_duration) {
        ros::spinOnce(); // wait for marker message
    }

    pose = geometry_msgs::Pose();
    return false;
}

void RepeatabilityMeasurementNode::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::MoveArmCommandResult::ConstPtr& result)
{
    AU_INFO("Received a Move Arm Command Result!");
    pending_command_ = false;
}
