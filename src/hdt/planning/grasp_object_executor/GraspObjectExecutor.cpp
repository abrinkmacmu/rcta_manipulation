#include "GraspObjectExecutor.h"

#include <eigen_conversions/eigen_msg.h>
#include <sbpl_geometry_utils/utils.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/common/stringifier/stringifier.h>
#include <hdt/common/msg_utils/msg_utils.h>
#include <hdt/control/robotiq_controllers/gripper_model.h>

namespace GraspObjectExecutionStatus
{

std::string to_string(Status status)
{
    switch (status) {
    case IDLE:
        return "Idle";
    case FAULT:
        return "Fault";
    case PLANNING_ARM_MOTION_TO_PREGRASP:
        return "PlanningArmMotionToPregrasp";
    case EXECUTING_ARM_MOTION_TO_PREGRASP:
        return "ExecutingArmMotionToPregrasp";
    case EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return "ExecutingVisualServoMotionToPregrasp";
    case EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return "ExecutingVisualServoMotionToGrasp";
    case GRASPING_OBJECT:
        return "GraspingObject";
    case PLANNING_ARM_MOTION_TO_STOW_POSITION:
        return "PlanningArmMotionToStowPosition";
    case EXECUTING_ARM_MOTION_TO_STOW_POSITION:
        return "ExecutingArmMotionToStowPosition";
    case COMPLETING_GOAL:
        return "CompletingGoal";
    default:
        return "Invalid";
    }
}

} // namespace GraspObjectExecutionStatus

GraspObjectExecutor::GraspObjectExecutor() :
    nh_(),
    ph_("~"),
    action_name_("grasp_object_command"),
    as_(),

    move_arm_command_action_name_("move_arm_command"),
    move_arm_command_client_(),
    generated_grasps_(false),
    sent_move_arm_goal_(false),
    pending_move_arm_command_(false),
    move_arm_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    move_arm_command_result_(),

    last_move_arm_pregrasp_goal_(),

    viservo_command_action_name_("viservo_command"),
    viservo_command_client_(),
    sent_viservo_command_(false),
    pending_viservo_command_(false),
    viservo_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    viservo_command_result_(),

    gripper_command_action_name_("gripper_controller/gripper_command_action"),
    gripper_command_client_(),
    sent_gripper_command_(false),
    pending_gripper_command_(false),
    gripper_command_goal_state_(actionlib::SimpleClientGoalState::SUCCEEDED),
    gripper_command_result_(),

    current_goal_(),
    status_(GraspObjectExecutionStatus::INVALID),
    last_status_(GraspObjectExecutionStatus::INVALID),
    grasp_spline_(),
    gas_can_scale_(0.1),
    wrist_to_tool_(),
    pregrasp_to_grasp_offset_m_(0.0),
    listener_()
{
}

bool GraspObjectExecutor::initialize()
{
    std::string urdf_string;
    if (!nh_.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return false;
    }

    robot_model_ = hdt::RobotModel::LoadFromURDF(urdf_string);
    if (!robot_model_) {
        ROS_ERROR("Failed to load Robot Model from the URDF");
        return false;
    }

    std::vector<geometry_msgs::Point> control_points;
    int degree;
    if (!msg_utils::download_param(ph_, "degree", degree) ||
        !msg_utils::download_param(ph_, "control_points", control_points))
    {
        ROS_ERROR("Failed to retrieve grasp spline parameters");
        return false;
    }

    std::vector<Eigen::Vector3d> grasp_spline_control_points(control_points.size());
    for (std::size_t i = 0; i < control_points.size(); ++i) {
        const geometry_msgs::Point& p = control_points[i];
        grasp_spline_control_points[i] = Eigen::Vector3d(p.x, p.y, p.z);
    }

    grasp_spline_.reset(new Nurb<Eigen::Vector3d>(grasp_spline_control_points, degree));
    if (!grasp_spline_) {
        ROS_ERROR("Failed to instantiate Nurb");
        return false;
    }

    ROS_INFO("Control Points:");
    for (const Eigen::Vector3d& control_vertex : grasp_spline_->control_points()) {
        ROS_INFO("    %s", to_string(control_vertex).c_str());
    }

    ROS_INFO("Knot Vector: %s", to_string(grasp_spline_->knots()).c_str());
    ROS_INFO("Degree: %s", std::to_string(grasp_spline_->degree()).c_str());

    geometry_msgs::Pose tool_pose_wrist_frame;
    if (!msg_utils::download_param(ph_, "wrist_to_tool_transform", tool_pose_wrist_frame)) {
        ROS_ERROR("Failed to retrieve 'wrist_to_tool_transform' from the param server");
        return false;
    }

    tf::poseMsgToEigen(tool_pose_wrist_frame, wrist_to_tool_);
    ROS_INFO("Wrist-to-Tool Transform: %s", to_string(wrist_to_tool_).c_str());

    if (!msg_utils::download_param(ph_, "pregrasp_to_grasp_offset_m", pregrasp_to_grasp_offset_m_)) {
        ROS_ERROR("Failed to retrieve 'pregrasp_to_grasp_offset_m' from the param server");
        return false;
    }

    grasp_to_pregrasp_ = Eigen::Translation3d(-pregrasp_to_grasp_offset_m_, 0, 0);

    marker_arr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);

    move_arm_command_client_.reset(new MoveArmCommandActionClient(move_arm_command_action_name_, false));
    if (!move_arm_command_client_) {
        ROS_ERROR("Failed to instantiate Move Arm Command Client");
        return false;
    }

    viservo_command_client_.reset(new ViservoCommandActionClient(viservo_command_action_name_, false));
    if (!viservo_command_client_) {
        ROS_ERROR("Failed to instantiate Viservo Command Client");
        return false;
    }

    gripper_command_client_.reset(new GripperCommandActionClient(gripper_command_action_name_, false));
    if (!gripper_command_client_) {
        ROS_ERROR("Failed to instantiate Gripper Command Client");
        return false;
    }

    as_.reset(new GraspObjectCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Grasp Object Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&GraspObjectExecutor::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&GraspObjectExecutor::preempt_callback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    return true;
}

int GraspObjectExecutor::run()
{
    if (!initialize()) {
        return FAILED_TO_INITIALIZE;
    }

    status_ = GraspObjectExecutionStatus::IDLE;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        RunUponDestruction rod([&](){ loop_rate.sleep(); });

        ros::spinOnce();

        if (status_ != last_status_) {
            ROS_INFO("Grasp Job Executor Transitioning: %s -> %s", to_string(last_status_).c_str(), to_string(status_).c_str());
            last_status_ = status_;
        }

        switch (status_) {
        case GraspObjectExecutionStatus::IDLE:
        {
            if (as_->isActive()) {
                status_ = GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP;
            }
        }   break;
        case GraspObjectExecutionStatus::FAULT:
        {
            if (as_->isActive()) {
                status_ = GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP;
            }
        }   break;
        case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP:
        {
            ////////////////////////////////////////////////////////////////////////////////
            // Executed upon entering PLANNING_ARM_MOTION_TO_PREGRASP
            ////////////////////////////////////////////////////////////////////////////////

            if (!generated_grasps_) {
                Eigen::Affine3d base_link_to_gas_canister;
                tf::poseMsgToEigen(current_goal_->gas_can_in_base_link.pose, base_link_to_gas_canister);

                int max_num_candidates = 100;
                const double min_u = 0.0;
                const double max_u = 1.0;

                // 1. generate grasp candidates (poses of the wrist in the robot frame) from the object pose
                std::vector<GraspCandidate> grasp_candidates;
                grasp_candidates.reserve(max_num_candidates);
                for (int i = 0; i < max_num_candidates; ++i) {
                    ROS_INFO("Candidate Grasp %3d", i);
                    // sample uniformly the position and derivative of the gas canister grasp spline
                    double u = (max_u - min_u) * i / (max_num_candidates - 1);

                    // TODO: skip the first "degree" (or order) knots at the beginning and end
                    int knot_num = -1;
                    for (int j = 0; j < grasp_spline_->knots().size() - 1; ++j) {
                        double curr_knot = grasp_spline_->knot(j);
                        double next_knot = grasp_spline_->knot(j + 1);
                        if (u >= curr_knot && u < next_knot) {
                            knot_num = j;
                            break;
                        }
                    }

                    if (knot_num < grasp_spline_->degree() ||
                        knot_num >= grasp_spline_->knots().size() - grasp_spline_->degree())
                    {
                        ROS_INFO("Skipping grasp_spline(%0.3f)", u);
                        continue;
                    }

                    Eigen::Vector3d sample_spline_point = (*grasp_spline_)(u);
                    ROS_INFO("    Sample Spline Point [canister frame]: %s", to_string(sample_spline_point).c_str());
                    Eigen::Vector3d sample_spline_deriv = grasp_spline_->deriv(u);
                    ROS_INFO("    Sample Spline Deriv [canister frame]: %s", to_string(sample_spline_deriv).c_str());

                    Eigen::Vector3d sample_spline_point_robot_frame =
                            base_link_to_gas_canister * Eigen::Scaling(gas_can_scale_) * sample_spline_point;
                    ROS_INFO("    Sample Spline Point [robot frame]: %s", to_string(sample_spline_point_robot_frame).c_str());

                    Eigen::Vector3d sample_spline_deriv_robot_frame =
                            base_link_to_gas_canister.rotation() * sample_spline_deriv.normalized();
                    ROS_INFO("    Sample Spline Deriv [robot frame]: %s", to_string(sample_spline_deriv_robot_frame).c_str());

                    // compute the normal to the grasp spline that most points "up" in the robot frame
                    Eigen::Vector3d up_bias(Eigen::Vector3d::UnitZ());
                    Eigen::Vector3d grasp_dir =
                            up_bias - up_bias.dot(sample_spline_deriv_robot_frame) * sample_spline_deriv_robot_frame;
                    grasp_dir.normalize();
                    grasp_dir *= -1.0;

                    ROS_INFO("Grasp Direction: %s", to_string(grasp_dir).c_str());

                    Eigen::Vector3d grasp_candidate_dir_x = grasp_dir;
                    Eigen::Vector3d grasp_candidate_dir_y = sample_spline_deriv_robot_frame;
                    Eigen::Vector3d grasp_candidate_dir_z = grasp_dir.cross(sample_spline_deriv_robot_frame);

                    Eigen::Matrix3d grasp_rotation_matrix;
                    grasp_rotation_matrix(0, 0) = grasp_candidate_dir_x.x();
                    grasp_rotation_matrix(1, 0) = grasp_candidate_dir_x.y();
                    grasp_rotation_matrix(2, 0) = grasp_candidate_dir_x.z();
                    grasp_rotation_matrix(0, 1) = grasp_candidate_dir_y.x();
                    grasp_rotation_matrix(1, 1) = grasp_candidate_dir_y.y();
                    grasp_rotation_matrix(2, 1) = grasp_candidate_dir_y.z();
                    grasp_rotation_matrix(0, 2) = grasp_candidate_dir_z.x();
                    grasp_rotation_matrix(1, 2) = grasp_candidate_dir_z.y();
                    grasp_rotation_matrix(2, 2) = grasp_candidate_dir_z.z();

                    // robot_frame -> candidate tool frame pose
                    Eigen::Affine3d grasp_candidate_rotation =
                            Eigen::Translation3d(sample_spline_point_robot_frame) * grasp_rotation_matrix;

                    // robot -> grasp candidate (desired tool) * tool -> wrist * wrist (grasp) -> pregrasp = robot -> wrist
                    Eigen::Affine3d candidate_wrist_transform = grasp_candidate_rotation * wrist_to_tool_.inverse() * grasp_to_pregrasp_;

                    ROS_INFO("Candidate Grasp Pose %d: %s", i, to_string(candidate_wrist_transform).c_str());

                    grasp_candidates.push_back(GraspCandidate(candidate_wrist_transform, u));
                }

                geometry_msgs::Vector3 triad_scale = geometry_msgs::CreateVector3(0.1, 0.01, 0.01);

                visualization_msgs::MarkerArray all_triad_markers;

                // create triad markers to be reused for each grasp candidate
                visualization_msgs::MarkerArray triad_markers = msg_utils::create_triad_marker_arr(triad_scale);
                for (visualization_msgs::Marker& marker : triad_markers.markers) {
                    marker.header.frame_id = current_goal_->gas_can_in_base_link.header.frame_id;
                    marker.ns = "candidate_grasp";
                }

                // save the original marker transforms
                std::vector<Eigen::Affine3d> marker_transforms;
                marker_transforms.reserve(triad_markers.markers.size());
                for (const auto& marker : triad_markers.markers) {
                    Eigen::Affine3d marker_transform;
                    tf::poseMsgToEigen(marker.pose, marker_transform);
                    marker_transforms.push_back(marker_transform);
                }

                int id = 0;
                for (const GraspCandidate& candidate : grasp_candidates) {
                    // transform triad markers to the candidate grasp transform
                    for (visualization_msgs::Marker& marker : triad_markers.markers) {
                        marker.id = id++;
                        Eigen::Affine3d marker_transform;
                        tf::poseMsgToEigen(marker.pose, marker_transform);
                        Eigen::Affine3d new_marker_transform = candidate.grasp_candidate_transform * marker_transform;
                        tf::poseEigenToMsg(new_marker_transform, marker.pose);
                    }

                    all_triad_markers.markers.insert(all_triad_markers.markers.end(), triad_markers.markers.begin(), triad_markers.markers.end());

                    // restore marker transforms
                    for (std::size_t i = 0; i < triad_markers.markers.size(); ++i) {
                        tf::poseEigenToMsg(marker_transforms[i], triad_markers.markers[i].pose);
                    }
                }

                ROS_INFO("Visualizing %zd triad markers", all_triad_markers.markers.size());
                marker_arr_pub_.publish(all_triad_markers);

                ROS_INFO("Sampled %zd grasp poses", grasp_candidates.size());

                // mount -> wrist = mount -> robot * robot -> wrist

                const std::string robot_frame = current_goal_->gas_can_in_base_link.header.frame_id;
                const std::string kinematics_frame = "arm_mount_panel_dummy";
                tf::StampedTransform tf_transform;
                try {
                    listener_.lookupTransform(robot_frame, kinematics_frame, ros::Time(0), tf_transform);
                }
                catch (const tf::TransformException& ex) {
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::PLANNING_FAILED;
                    std::stringstream ss;
                    ss << "Failed to lookup transform " << robot_frame << " -> " << kinematics_frame << "; Unable to determine grasp reachability";
                    as_->setAborted(result, ss.str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                Eigen::Affine3d robot_to_kinematics;
                msg_utils::convert(tf_transform, robot_to_kinematics);

                // 2. filter unreachable grasp candidates
                reachable_grasp_candidates_.clear();
                reachable_grasp_candidates_.reserve(grasp_candidates.size());
                for (const GraspCandidate& grasp_candidate : grasp_candidates) {
                    Eigen::Affine3d kinematics_to_grasp_candidate =
                            robot_to_kinematics.inverse() * grasp_candidate.grasp_candidate_transform;

                    std::vector<double> fake_seed(robot_model_->joint_names().size(), 0.0);
                    std::vector<double> sol;
                    if (robot_model_->search_nearest_ik(
                            kinematics_to_grasp_candidate, fake_seed, sol, sbpl::utils::ToRadians(1.0)))
                    {
                        GraspCandidate reachable_grasp_candidate(kinematics_to_grasp_candidate, grasp_candidate.u);
                        reachable_grasp_candidates_.push_back(reachable_grasp_candidate);
                    }
                }

                ROS_INFO("Produced %zd reachable grasp poses", reachable_grasp_candidates_.size());

                if (reachable_grasp_candidates_.empty()) {
                    ROS_WARN("No reachable grasp candidates available");
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::OBJECT_OUT_OF_REACH;
                    as_->setAborted(result, "No reachable grasp candidates available");
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                // 3. sort grasp candidates by desirability (note: more desirable grasps are at the end of the vector)
                std::sort(reachable_grasp_candidates_.begin(), reachable_grasp_candidates_.end(),
                          [&](const GraspCandidate& a, const GraspCandidate& b) -> bool
                          {
                              double mid_u = 0.5 * (min_u + max_u);
                              return fabs(a.u - mid_u) > fabs(b.u - mid_u);
                          });

                generated_grasps_ = true;
            }

            ////////////////////////////////////////////////////////////////////////////////
            // Main loop of PLANNING_ARM_MOTION_TO_PREGRASP
            ////////////////////////////////////////////////////////////////////////////////

            if (!sent_move_arm_goal_) {
                hdt::GraspObjectCommandFeedback feedback;
                feedback.status = execution_status_to_feedback_status(status_);
                as_->publishFeedback(feedback);

                if (reachable_grasp_candidates_.empty()) {
                    ROS_WARN("Failed to plan to all reachable grasps");
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::PLANNING_FAILED;
                    as_->setAborted(result, "Failed to plan to all reachable grasps");
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                ROS_WARN("Sending Move Arm Goal to pregrasp pose");
                if (!wait_for_action_server(
                        move_arm_command_client_,
                        move_arm_command_action_name_,
                        ros::Duration(0.1),
                        ros::Duration(5.0)))
                {
                    std::stringstream ss; ss << "Failed to connect to '" << move_arm_command_action_name_ << "' action server";
                    ROS_WARN("%s", ss.str().c_str());
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::PLANNING_FAILED;
                    as_->setAborted(result, ss.str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                const GraspCandidate& next_best_grasp = reachable_grasp_candidates_.back();

                // 4. send a move arm goal for the best grasp
                last_move_arm_pregrasp_goal_.type = hdt::MoveArmCommandGoal::EndEffectorGoal;
                tf::poseEigenToMsg(next_best_grasp.grasp_candidate_transform, last_move_arm_pregrasp_goal_.goal_pose);

                auto result_cb = boost::bind(&GraspObjectExecutor::move_arm_command_result_cb, this, _1, _2);
                move_arm_command_client_->sendGoal(last_move_arm_pregrasp_goal_, result_cb);

                pending_move_arm_command_ = true;
                sent_move_arm_goal_ = true;
            }
            else if (!pending_move_arm_command_) {
                // NOTE: short-circuiting "EXECUTING_ARM_MOTION_TO_PREGRASP" for
                // now since the move_arm action handles execution and there is
                // presently no feedback to distinguish planning vs. execution

                ROS_INFO("Move Arm Goal is no longer pending");
                if (move_arm_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    move_arm_command_result_ && move_arm_command_result_->success)
                {
                    ROS_INFO("Move Arm Command succeeded");
                    status_ = GraspObjectExecutionStatus::OPENING_GRIPPER;
                }
                else {
                    ROS_INFO("Move Arm Command failed");
                    ROS_INFO("    Simple Client Goal State: %s", move_arm_command_goal_state_.toString().c_str());
                    ROS_INFO("    Error Text: %s", move_arm_command_goal_state_.getText().c_str());
                    ROS_INFO("    result.success = %s", move_arm_command_result_ ? (move_arm_command_result_->success ? "TRUE" : "FALSE") : "null");
                    // stay in PLANNING_ARM_MOTION_TO_PREGRASP until there are no more grasps
                    // TODO: consider moving back to the stow position
                }

                sent_move_arm_goal_ = false; // reset for future move arm goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_PREGRASP:
        {
        }   break;
        case GraspObjectExecutionStatus::OPENING_GRIPPER:
        {
            if (!sent_gripper_command_) {
                ROS_WARN("Sending Gripper Goal to open gripper");
                if (!wait_for_action_server(
                        gripper_command_client_,
                        gripper_command_action_name_,
                        ros::Duration(0.1),
                        ros::Duration(5.0)))
                {
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::EXECUTION_FAILED;
                    std::stringstream ss; ss << "Failed to connect to '" << gripper_command_action_name_ << "' action server";
                    as_->setAborted(result, ss.str());
                    ROS_WARN("%s", ss.str().c_str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                control_msgs::GripperCommandGoal gripper_goal;
                gripper_goal.command.max_effort = GripperModel().maximum_force();
                gripper_goal.command.position = GripperModel().maximum_width();

                auto result_cb = boost::bind(&GraspObjectExecutor::gripper_command_result_cb, this, _1, _2);
                gripper_command_client_->sendGoal(gripper_goal, result_cb);

                pending_gripper_command_ = true;
                sent_gripper_command_ = true;
            }
            if (!pending_gripper_command_) {
                ROS_INFO("Gripper Goal to open gripper is no longer pending");

                if (gripper_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    gripper_command_result_->reached_goal)
                {
                    ROS_INFO("Gripper Command Succeeded");
                    status_ = GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
                }
                else {
                    ROS_INFO("Gripper Command failed");
                    ROS_INFO("    Simple Client Goal State: %s", gripper_command_goal_state_.toString().c_str());
                    ROS_INFO("    Error Text: %s", gripper_command_goal_state_.getText().c_str());
                    ROS_INFO("    result.reached_goal = %s",
                            gripper_command_result_ ? (gripper_command_result_->reached_goal ? "TRUE" : "FALSE") : "null");
                }

                sent_gripper_command_ = false; // reset for future gripper goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        {
            if (!sent_viservo_command_) {
                ROS_WARN("Sending Viservo Goal to pregrasp pose");

                if (!wait_for_action_server(
                        viservo_command_client_,
                        viservo_command_action_name_,
                        ros::Duration(0.1),
                        ros::Duration(5.0)))
                {
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::EXECUTION_FAILED;
                    std::stringstream ss; ss << "Failed to connect to '" << viservo_command_action_name_ << "' action server";
                    as_->setAborted(result, ss.str());
                    ROS_WARN("%s", ss.str().c_str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                const std::string kinematics_frame = "arm_mount_panel_dummy";
                const std::string camera_frame = "asus_rgb_frame";

                tf::StampedTransform tf_transform;
                try {
                    listener_.lookupTransform(camera_frame, kinematics_frame, ros::Time(0), tf_transform);
                }
                catch (const tf::TransformException& ex) {
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::EXECUTION_FAILED;
                    std::stringstream ss;
                    ss << "Failed to lookup transform " << kinematics_frame << " -> " << camera_frame << "; Unable to determine viservo goal";
                    as_->setAborted(result, ss.str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                Eigen::Affine3d camera_to_kinematics;
                msg_utils::convert(tf_transform, camera_to_kinematics);

                // camera -> kinematics * kinematics -> wrist
                Eigen::Affine3d kinematics_to_wrist;
                tf::poseMsgToEigen(last_move_arm_pregrasp_goal_.goal_pose, kinematics_to_wrist);

                Eigen::Affine3d viservo_pregrasp_goal_transform =
                        camera_to_kinematics * kinematics_to_wrist;

                last_viservo_pregrasp_goal_.goal_id = current_goal_->id;
                tf::poseEigenToMsg(viservo_pregrasp_goal_transform, last_viservo_pregrasp_goal_.goal_pose);

                auto result_cb = boost::bind(&GraspObjectExecutor::viservo_command_result_cb, this, _1, _2);
                viservo_command_client_->sendGoal(last_viservo_pregrasp_goal_, result_cb);

                pending_viservo_command_ = true;
                sent_viservo_command_ = true;
            }
            else if (!pending_viservo_command_) {
                ROS_INFO("Viservo Goal to pregrasp is no longer pending");

                if (viservo_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    viservo_command_result_->result == hdt::ViservoCommandResult::SUCCESS)
                {
                    ROS_INFO("Viservo Command Succeeded");
                    status_ = GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
                }
                else {
                    ROS_INFO("Viservo Command failed");
                    ROS_INFO("    Simple Client Goal State: %s", viservo_command_goal_state_.toString().c_str());
                    ROS_INFO("    Error Text: %s", viservo_command_goal_state_.getText().c_str());
                    ROS_INFO("    result.result = %s",
                            viservo_command_result_ ?
                                (viservo_command_result_->result == hdt::ViservoCommandResult::SUCCESS? "SUCCESS" : "NOT SUCCESS (lol)") :
                                "null");
                }

                sent_viservo_command_ = false; // reset for future viservo goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        {
            if (!sent_viservo_command_) {
                ROS_WARN("Sending Viservo Goal to grasp pose");

                if (!wait_for_action_server(
                        viservo_command_client_,
                        viservo_command_action_name_,
                        ros::Duration(0.1),
                        ros::Duration(5.0)))
                {
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::EXECUTION_FAILED;
                    std::stringstream ss; ss << "Failed to connect to '" << viservo_command_action_name_ << "' action server";
                    as_->setAborted(result, ss.str());
                    ROS_WARN("%s", ss.str().c_str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                Eigen::Affine3d viservo_grasp_goal_transform;
                tf::poseMsgToEigen(last_viservo_pregrasp_goal_.goal_pose, viservo_grasp_goal_transform);
                // camera -> pregrasp * pregrasp -> grasp = camera -> grasp
                viservo_grasp_goal_transform = viservo_grasp_goal_transform * grasp_to_pregrasp_.inverse();

                last_viservo_grasp_goal_.goal_id = current_goal_->id;
                tf::poseEigenToMsg(viservo_grasp_goal_transform, last_viservo_grasp_goal_.goal_pose);

                auto result_cb = boost::bind(&GraspObjectExecutor::viservo_command_result_cb, this, _1, _2);
                viservo_command_client_->sendGoal(last_viservo_grasp_goal_, result_cb);

                pending_viservo_command_ = true;
                sent_viservo_command_ = true;;
            }
            else if (!pending_viservo_command_) {
                ROS_INFO("Viservo Goal to grasp is no longer pending");

                if (viservo_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    viservo_command_result_->result == hdt::ViservoCommandResult::SUCCESS)
                {
                    ROS_INFO("Viservo Command Succeeded");
                    status_ = GraspObjectExecutionStatus::GRASPING_OBJECT;
                }
                else {
                    ROS_INFO("Viservo Command failed");
                    ROS_INFO("    Simple Client Goal State: %s", viservo_command_goal_state_.toString().c_str());
                    ROS_INFO("    Error Text: %s", viservo_command_goal_state_.getText().c_str());
                    ROS_INFO("    result.result = %s",
                            viservo_command_result_ ?
                                (viservo_command_result_->result == hdt::ViservoCommandResult::SUCCESS? "SUCCESS" : "NOT SUCCESS (lol)") :
                                "null");
                }

                sent_viservo_command_ = false; // reset for future viservo goals
            }
        }   break;
        case GraspObjectExecutionStatus::GRASPING_OBJECT:
        {
            if (!sent_gripper_command_) {
                ROS_WARN("Sending Gripper Goal to close gripper");
                if (!wait_for_action_server(
                        gripper_command_client_,
                        gripper_command_action_name_,
                        ros::Duration(0.1),
                        ros::Duration(5.0)))
                {
                    hdt::GraspObjectCommandResult result;
                    result.result = hdt::GraspObjectCommandResult::EXECUTION_FAILED;
                    std::stringstream ss; ss << "Failed to connect to '" << gripper_command_action_name_ << "' action server";
                    as_->setAborted(result, ss.str());
                    ROS_WARN("%s", ss.str().c_str());
                    status_ = GraspObjectExecutionStatus::FAULT;
                    break;
                }

                control_msgs::GripperCommandGoal gripper_goal;
                gripper_goal.command.max_effort = GripperModel().maximum_force();
                gripper_goal.command.position = GripperModel().minimum_width();

                auto result_cb = boost::bind(&GraspObjectExecutor::gripper_command_result_cb, this, _1, _2);
                gripper_command_client_->sendGoal(gripper_goal, result_cb);

                pending_gripper_command_ = true;
                sent_gripper_command_ = true;
            }
            if (!pending_gripper_command_) {
                ROS_INFO("Gripper Goal to close gripper is no longer pending");

                if (gripper_command_goal_state_ == actionlib::SimpleClientGoalState::SUCCEEDED &&
                    gripper_command_result_->reached_goal)
                {
                    ROS_INFO("Gripper Command Succeeded");
                    status_ = GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
                }
                else {
                    ROS_INFO("Gripper Command failed");
                    ROS_INFO("    Simple Client Goal State: %s", gripper_command_goal_state_.toString().c_str());
                    ROS_INFO("    Error Text: %s", gripper_command_goal_state_.getText().c_str());
                    ROS_INFO("    result.reached_goal = %s",
                            gripper_command_result_ ? (gripper_command_result_->reached_goal ? "TRUE" : "FALSE") : "null");
                }

                sent_gripper_command_ = false; // reset for future gripper goals
            }
        }   break;
        case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_STOW_POSITION:
        {
            if (!sent_move_arm_goal_) {
                ROS_INFO("Sending Move Arm Goal to stow position");
//                pending_move_arm_command_ = true;
                sent_move_arm_goal_ = true;
            }
            else if (!pending_move_arm_command_) {
                // NOTE: short-circuiting
                // "EXECUTING_ARM_MOTION_TO_STOW_POSITION" for now since hte
                // move arm action handles execution and there is presently no
                // feedback to distinguish planning vs. execution
                status_ = GraspObjectExecutionStatus::COMPLETING_GOAL;
                sent_move_arm_goal_ = false; // reset for future move arm goals
            }
        }   break;
        case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_STOW_POSITION:
        {
        }   break;
        case GraspObjectExecutionStatus::COMPLETING_GOAL:
        {
            hdt::GraspObjectCommandResult result;
            result.result = hdt::GraspObjectCommandResult::SUCCESS;
            as_->setSucceeded(result);
            status_ = GraspObjectExecutionStatus::IDLE;
        }   break;
        default:
            break;
        }
    }

    return SUCCESS;
}

void GraspObjectExecutor::goal_callback()
{
    ROS_INFO("Received a new goal");
    current_goal_ = as_->acceptNewGoal();
    ROS_INFO("    Goal ID: %u", current_goal_->id);
    ROS_INFO("    Retry Count: %d", current_goal_->retryCount);
    ROS_INFO("    Gas Can Pose [map]", to_string(current_goal_->gas_can_in_map.pose).c_str());
    ROS_INFO("    Gas Can Pose [base_link]", to_string(current_goal_->gas_can_in_base_link.pose).c_str());
    ROS_INFO("    Octomap ID: %s", current_goal_->octomap.id.c_str());

    generated_grasps_ = false;
    sent_move_arm_goal_ = false;
    pending_move_arm_command_ = false;

    sent_viservo_command_ = false;
    pending_viservo_command_ = false;

    sent_gripper_command_ = false;
    pending_gripper_command_ = false;
}

void GraspObjectExecutor::preempt_callback()
{

}

void GraspObjectExecutor::move_arm_command_active_cb()
{

}

void GraspObjectExecutor::move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback)
{

}

void GraspObjectExecutor::move_arm_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::MoveArmCommandResult::ConstPtr& result)
{
    move_arm_command_goal_state_ = state;
    move_arm_command_result_ = result;
    pending_move_arm_command_ = false;
}

void GraspObjectExecutor::viservo_command_active_cb()
{

}

void GraspObjectExecutor::viservo_command_feedback_cb(const hdt::ViservoCommandFeedback::ConstPtr& feedback)
{

}

void GraspObjectExecutor::viservo_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::ViservoCommandResult::ConstPtr& result)
{
    viservo_command_goal_state_ = state;
    viservo_command_result_ = result;
    pending_viservo_command_ = false;
}

void GraspObjectExecutor::gripper_command_active_cb()
{

}

void GraspObjectExecutor::gripper_command_feedback_cb(const control_msgs::GripperCommandFeedback::ConstPtr& feedback)
{
}

void GraspObjectExecutor::gripper_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const control_msgs::GripperCommandResult::ConstPtr& result)
{
    gripper_command_goal_state_ = state;
    gripper_command_result_ = result;
    pending_gripper_command_ = false;
}

uint8_t GraspObjectExecutor::execution_status_to_feedback_status(GraspObjectExecutionStatus::Status status)
{
    switch (status) {
    case GraspObjectExecutionStatus::IDLE:
        return -1;
    case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_PREGRASP:
        return hdt::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_PREGRASP:
        return hdt::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP:
        return hdt::GraspObjectCommandFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_PREGRASP;
    case GraspObjectExecutionStatus::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP:
        return hdt::GraspObjectCommandFeedback::EXECUTING_VISUAL_SERVO_MOTION_TO_GRASP;
    case GraspObjectExecutionStatus::GRASPING_OBJECT:
        return hdt::GraspObjectCommandFeedback::GRASPING_OBJECT;
    case GraspObjectExecutionStatus::PLANNING_ARM_MOTION_TO_STOW_POSITION:
        return hdt::GraspObjectCommandFeedback::PLANNING_ARM_MOTION_TO_STOW;
    case GraspObjectExecutionStatus::EXECUTING_ARM_MOTION_TO_STOW_POSITION:
        return hdt::GraspObjectCommandFeedback::EXECUTING_ARM_MOTION_TO_STOW;
    case GraspObjectExecutionStatus::COMPLETING_GOAL:
        return -1;
    default:
        return -1;
    }
}
