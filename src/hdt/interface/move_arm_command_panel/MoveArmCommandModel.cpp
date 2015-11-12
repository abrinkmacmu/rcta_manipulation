#include "MoveArmCommandModel.h"

#include <stack>

#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <ros/console.h>
#include <sbpl_geometry_utils/utils.h>

static std::string to_string(moveit_msgs::MoveItErrorCodes code)
{
    switch (code.val) {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
        return "SUCCESS";
    case moveit_msgs::MoveItErrorCodes::FAILURE:
        return "FAILURE";

    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
        return "PLANNING_FAILED";
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        return "INVALID_MOTION_PLAN";
    case moveit_msgs::MoveItErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case moveit_msgs::MoveItErrorCodes::CONTROL_FAILED:
        return "CONTROL_FAILED";
    case moveit_msgs::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case moveit_msgs::MoveItErrorCodes::TIMED_OUT:
        return "TIMED_OUT";
    case moveit_msgs::MoveItErrorCodes::PREEMPTED:
        return "PREEMPTED";

    case moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION:
        return "START_STATE_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "START_STATE_VIOLATES_PATH_CONSTRAINTS";

    case moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION:
        return "GOAL_IN_COLLISION";
    case moveit_msgs::MoveItErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::GOAL_CONSTRAINTS_VIOLATED:
        return "GOAL_CONSTRAINTS_VIOLATED";

    case moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME:
        return "INVALID_GROUP_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS:
        return "INVALID_GOAL_CONSTRAINTS";
    case moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE:
        return "INVALID_ROBOT_STATE";
    case moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME:
        return "INVALID_LINK_NAME";
    case moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME:
        return "INVALID_OBJECT_NAME";

    case moveit_msgs::MoveItErrorCodes::FRAME_TRANSFORM_FAILURE:
        return "FRAME_TRANSFORM_FAILURE";
    case moveit_msgs::MoveItErrorCodes::COLLISION_CHECKING_UNAVAILABLE:
        return "COLLISION_CHECKING_UNAVAILABLE";
    case moveit_msgs::MoveItErrorCodes::ROBOT_STATE_STALE:
        return "ROBOT_STATE_STALE";
    case moveit_msgs::MoveItErrorCodes::SENSOR_INFO_STALE:
        return "SENSOR_INFO_STALE";

    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
        return "NO_IK_SOLUTION";

    default:
        return "UNRECOGNIZED";
    }
}

const double MoveArmCommandModel::DefaultTableSizeX = 1.0;
const double MoveArmCommandModel::DefaultTableSizeY = 2.0;
const double MoveArmCommandModel::DefaultTableSizeZ = 0.2;

MoveArmCommandModel::MoveArmCommandModel(QObject* parent) :
    QObject(parent),
    m_nh(),
    m_robot_description(),
    m_rm_loader(),
    m_robot_model(),
    m_robot_state(),
    m_joint_states_sub(),
    m_last_joint_state_msg(),
    m_plan_path_client(),
    m_table_size_x(DefaultTableSizeX),
    m_table_size_y(DefaultTableSizeY),
    m_table_size_z(DefaultTableSizeZ),
    m_table_origin_x(1.0),
    m_table_origin_y(0.0),
    m_table_origin_z(0.48),
    m_table_name("table"),
    m_planning_scene_world_pub(),
    m_collision_object_pub()
{
    m_joint_states_sub = m_nh.subscribe(
            "joint_states", 5, &MoveArmCommandModel::jointStatesCallback, this);
    m_plan_path_client = m_nh.serviceClient<moveit_msgs::GetMotionPlan>(
            "plan_kinematic_path");

    m_planning_scene_world_pub = m_nh.advertise<moveit_msgs::PlanningSceneWorld>(
            "planning_scene_world", 5, true);
    m_collision_object_pub = m_nh.advertise<moveit_msgs::CollisionObject>(
            "collision_object", 5, true);

    // publish an empty planning scene world
    moveit_msgs::PlanningSceneWorld planning_scene_world;
//    m_planning_scene_world_pub.publish(planning_scene_world);

    // publish the table
    moveit_msgs::CollisionObject table_collision_object =
            createTableCollisionObject();
//    m_collision_object_pub.publish(table_collision_object);
}

bool MoveArmCommandModel::loadRobot(const std::string& robot_description)
{
    if (robot_description == m_robot_description) {
        return true;
    }

    // load a new robot
    robot_model_loader::RobotModelLoaderPtr rm_loader(
            new robot_model_loader::RobotModelLoader(robot_description, true));

    if (!rm_loader->getModel()) {
        // failed to load robot from URDF
        return false;
    }

    m_robot_description = robot_description;
    m_rm_loader = rm_loader;
    m_robot_model = m_rm_loader->getModel();
    m_robot_state.reset(new moveit::core::RobotState(m_robot_model));

    m_robot_state->setToDefaultValues();
    m_robot_state->updateLinkTransforms();
    m_robot_state->updateCollisionBodyTransforms();

    logRobotModelInfo(*m_robot_model);

    clearMoveGroupRequest();

    Q_EMIT robotLoaded();
    return true;
}

bool MoveArmCommandModel::isRobotLoaded() const
{
    return (bool)m_robot_model.get();
}

const std::string& MoveArmCommandModel::robotDescription() const
{
    return m_robot_description;
}

moveit::core::RobotModelConstPtr MoveArmCommandModel::robotModel() const
{
    return m_robot_model;
}

moveit::core::RobotStateConstPtr MoveArmCommandModel::robotState() const
{
    return m_robot_state;
}

std::map<std::string, double>
MoveArmCommandModel::getRightArmTorques(
    double fx, double fy, double fz,
    double ta, double tb, double tc) const
{
    // T = J^T() * forces;
    std::map<std::string, double> torques;

    const moveit::core::JointModelGroup* jmg =
            m_robot_model->getJointModelGroup("right_arm");

    Eigen::MatrixXd J;
    J = m_robot_state->getJacobian(jmg, Eigen::Vector3d(0.17, 0.0, 0.0));

    ROS_DEBUG_STREAM("Jacobian = " << J.rows() << " x " << J.cols() << '\n' << J);

    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d f;
    f(0) = fx;
    f(1) = fy;
    f(2) = fz;
    f(3) = ta;
    f(4) = tb;
    f(5) = tc;

    Eigen::VectorXd t = J.transpose() * f;

    ROS_INFO("Torques = (%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f)", t(0), t(1), t(2), t(3), t(4), t(5), t(6));

    const std::vector<std::string> rarm_joint_names =
    {
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint"
    };

    torques =
    {
        { rarm_joint_names[0], t(0) },
        { rarm_joint_names[1], t(1) },
        { rarm_joint_names[2], t(2) },
        { rarm_joint_names[3], t(3) },
        { rarm_joint_names[4], t(4) },
        { rarm_joint_names[5], t(5) },
        { rarm_joint_names[6], t(6) },
    };

    return torques;
}

bool MoveArmCommandModel::readyToPlan() const
{
    if (!isRobotLoaded()) {
        ROS_WARN("Cannot plan with no robot loaded");
        return false;
    }

    if (!m_last_joint_state_msg) {
        ROS_WARN("Haven't received a JointState message yet");
        return false;
    }

    return true;
}

bool MoveArmCommandModel::planToPosition(const std::string& group_name)
{
    if (!readyToPlan()) {
        return false;
    }

    moveit_msgs::GetMotionPlan srv;
    moveit_msgs::MotionPlanRequest& req = srv.request.motion_plan_request;

    const ros::Time now = ros::Time::now();

    if (!fillWorkspaceParameters(now, group_name, req) ||
        !fillStartState(now, group_name, req) ||
        !fillGoalConstraints(now, group_name, req) ||
        !fillPathConstraints(now, group_name, req) ||
        !fillTrajectoryConstraints(now, group_name, req))
    {
        return false;
    }

    req.planner_id = "ARA*";
    req.group_name = group_name;
    req.num_planning_attempts = 1;
    req.allowed_planning_time = 10.0;
    req.max_velocity_scaling_factor = 1.0;

    if (!m_plan_path_client.call(srv)) {
        ROS_ERROR("Service call failed");
        return false;
    }

    const moveit_msgs::MotionPlanResponse& res = srv.response.motion_plan_response;
    ROS_INFO("Service call returned with code '%s", to_string(res.error_code).c_str());
    ROS_INFO("trajectory_start:");
    ROS_INFO("group_name: %s", res.group_name.c_str());
    ROS_INFO("trajectory:");
    ROS_INFO("  joint_trajectory: ");
    ROS_INFO("    header: ");
    ROS_INFO("      seq: ");
    ROS_INFO("      stamp: ");
    ROS_INFO("      frame_id: %s", res.trajectory.joint_trajectory.header.frame_id.c_str());
    ROS_INFO("    joint_names: %zu", res.trajectory.joint_trajectory.joint_names.size());
    ROS_INFO("    points: %zu", res.trajectory.joint_trajectory.points.size());
    ROS_INFO("      positions: ");
    ROS_INFO("      velocities: ");
    ROS_INFO("      accelerations: ");
    ROS_INFO("      effort: ");
    ROS_INFO("      time_from_start: ");
    ROS_INFO("  multi_dof_joint_trajectory: ");
    ROS_INFO("    joint_names: %zu", res.trajectory.multi_dof_joint_trajectory.joint_names.size());
    ROS_INFO("    points: %zu", res.trajectory.multi_dof_joint_trajectory.points.size());
    ROS_INFO("      transforms: ");
    ROS_INFO("      velocities: ");
    ROS_INFO("      accelerations: ");
    ROS_INFO("      time_from_start: ");
    ROS_INFO("planning_time: %0.6f", res.planning_time);
    ROS_INFO("error_code: { val: %s }", to_string(res.error_code).c_str());

    return true;
}

bool MoveArmCommandModel::copyCurrentState()
{
    if (!m_last_joint_state_msg) {
        return false;
    }

    m_robot_state->setVariableValues(*m_last_joint_state_msg);
    Q_EMIT robotStateChanged();
}

void MoveArmCommandModel::setJointVariable(int jidx, double value)
{
    if (!isRobotLoaded()) {
        return;
    }

    if (jidx < 0 || jidx >= m_robot_model->getVariableCount()) {
        ROS_WARN("Index passed to setJointVariable out of bounds: jidx = %d, variable count = %zu", jidx, m_robot_model->getVariableCount());
        return;
    }

    if (m_robot_state->getVariablePosition(jidx) != value) {
        m_robot_state->setVariablePosition(jidx, value);
        if (m_robot_state->getVariablePosition(jidx) != value) {
            ROS_WARN("Attempt to set joint variable %d to %0.3f failed", jidx, value);
        }
        m_robot_state->updateLinkTransforms();
        m_robot_state->updateCollisionBodyTransforms();
        Q_EMIT robotStateChanged();
    }
}

void MoveArmCommandModel::logRobotModelInfo(
    const moveit::core::RobotModel& rm) const
{                                                                                                                             
    ROS_INFO("Robot Model Name: %s", rm.getName().c_str());                                                                   
    ROS_INFO("Robot Model Frame: %s", rm.getModelFrame().c_str());                                                            
    ROS_INFO("Root Link Name: %s", rm.getRootLinkName().c_str());                                                             
    ROS_INFO("Root Joint Name: %s", rm.getRootJointName().c_str());                                                           
                                                                                                                              
    ROS_INFO("--- Robot Links ---");                                                                                          
    std::stack<std::pair<int, const moveit::core::LinkModel*>> links;                                                         
    links.push(std::make_pair(0, rm.getRootLink()));                                                                          
    while (!links.empty()) {                                                                                                  
        int depth;                                                                                                            
        const moveit::core::LinkModel* lm;                                                                                    
        std::tie(depth, lm) = links.top();                                                                                    
        links.pop();                                                                                                          
                                                                                                                              
        std::string pad(depth, ' ');                                                                                          
        ROS_INFO("%s%s", pad.c_str(), lm->getName().c_str());                                                                 
                                                                                                                              
        for (const moveit::core::JointModel* jm : lm->getChildJointModels()) {                                                
            links.push(std::make_pair(depth+1, jm->getChildLinkModel()));                                                     
        }                                                                                                                     
    }                                                                                                                         
                                                                                                                              
    ROS_INFO("--- Robot Joints ---");                                                                                         
    std::stack<std::pair<int, const moveit::core::JointModel*>> joints;                                                       
    joints.push(std::make_pair(0, rm.getRootJoint()));                                                                        
    while (!joints.empty()) {                                                                                                 
        int depth;                                                                                                            
        const moveit::core::JointModel* jm;                                                                                   
        std::tie(depth, jm) = joints.top();                                                                                   
        joints.pop();                                                                                                         
                                                                                                                              
        std::string pad(depth, ' ');                                                                                          
        ROS_INFO("%s%s", pad.c_str(), jm->getName().c_str());                                                                 
                                                                                                                              
        const moveit::core::LinkModel* lm = jm->getChildLinkModel();                                                          
        for (const moveit::core::JointModel* j : lm->getChildJointModels()) {                                                 
            joints.push(std::make_pair(depth + 1, j));                                                                        
        }                                                                                                                     
    }                                                                                                                         
                                                                                                                              
    ROS_INFO("--- Robot Joint Groups ---");                                                                                   
                                                                                                                              
    const std::vector<const moveit::core::JointModelGroup*>& jmgs =                                                           
            rm.getJointModelGroups();                                                                                         
    for (const moveit::core::JointModelGroup* jmg : jmgs) {                                                                   
        ROS_INFO("Name: %s", jmg->getName().c_str());                                                                         
        ROS_INFO("  Joints:");                                                                                                
        for (const std::string& name : jmg->getJointModelNames()) {                                                           
            ROS_INFO("    %s", name.c_str());                                                                                 
        }                                                                                                                     
        ROS_INFO("  Chain: %s", jmg->isChain() ? "true" : "false");                                                           
        ROS_INFO("  Only Single-DoF Joints: %s", jmg->isSingleDOFJoints() ? "true" : "false");                                
        ROS_INFO("  End Effector: %s", jmg->isEndEffector() ? "true" : "false");                                              
    }                                                                                                                         

    ROS_INFO("--- Joint Variables ---");

    for (size_t vind = 0; vind < rm.getVariableCount(); ++vind) {
        const std::string& var_name = rm.getVariableNames()[vind];
        const auto& var_bounds = rm.getVariableBounds(var_name);
        ROS_INFO("%s: { min: %f, max: %f, vel: %f, acc: %f }", var_name.c_str(), var_bounds.min_position_, var_bounds.max_position_, var_bounds.max_velocity_, var_bounds.max_acceleration_);
    }
}

void MoveArmCommandModel::jointStatesCallback(
    const sensor_msgs::JointState::ConstPtr& msg)
{
    m_last_joint_state_msg = msg;
}

void MoveArmCommandModel::clearMoveGroupRequest()
{
    // workspace parameters dependent on the robot model
    // start-state possibly dependent on the robot model
    // constraints dependent on the robot model
    // planner id dependent on plugins loaded into move_group
    // group_name dependent on the loaded robot loaded
    // planning attempts, planning time, and scaling factor not-dependent on anything
}

bool MoveArmCommandModel::fillWorkspaceParameters(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req)
{
//    req.workspace_parameters.header.frame_id = m_robot_model->getModelFrame();
//    req.workspace_parameters.header.seq = 0;
//    req.workspace_parameters.header.stamp = now;
//    req.workspace_parameters.min_corner.x = -0.5;
//    req.workspace_parameters.min_corner.y = -1.0;
//    req.workspace_parameters.min_corner.z = 0.0;
//    req.workspace_parameters.max_corner.x = 1.0;
//    req.workspace_parameters.max_corner.y = 1.0;
//    req.workspace_parameters.max_corner.z = 3.0;

    req.workspace_parameters.header.frame_id = "torso_lift_link";
    req.workspace_parameters.header.seq = 0;
    req.workspace_parameters.header.stamp = now;
    req.workspace_parameters.min_corner.x = 0.0;
    req.workspace_parameters.min_corner.y = -1.0;
    req.workspace_parameters.min_corner.z = -2.0;
    req.workspace_parameters.max_corner.x = 1.5;
    req.workspace_parameters.max_corner.y = 1.0;
    req.workspace_parameters.max_corner.z = 1.0;
    return true;
}

bool MoveArmCommandModel::fillStartState(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    // copy over joint state from incoming sensor data
    req.start_state.joint_state.header.frame_id = "ooga booga";
    req.start_state.joint_state.header.seq = 0;
    req.start_state.joint_state.header.stamp = now;
    req.start_state.joint_state.name = m_last_joint_state_msg->name;
    req.start_state.joint_state.position = m_last_joint_state_msg->position;
    req.start_state.joint_state.velocity = m_last_joint_state_msg->velocity;
    req.start_state.joint_state.effort = m_last_joint_state_msg->effort;

    // TODO: keep another robot state around as the "current robot state" and
    // use it as the single point of access, rather than distinguishing here
    // between live sources of state and sources of state from the phantom

    // copy over joint

    // for each multi-dof joint
    //   find the transform between the parent link and the child link and set
    //   the transform accordingly here
    sensor_msgs::MultiDOFJointState& multi_dof_joint_state =
            req.start_state.multi_dof_joint_state;

    // WARN: So, I'm not really sure why a frame id is necessary here. I was
    // under the impression the multi-DOF transform would be the transform from
    // the parent link to the child link (which, I believe, agrees with
    // RobotState::getJointTransform), but there's some weird stuff going on in
    // moveit/robot_state/conversions.h that leads to believe maybe somewhere is
    // assuming the transform is from the model frame to the child link. Either
    // way, we're going to set the header of the joint state to be the model
    // frame, to avoid that conversion, and then be happy with the fact that the
    // only multi-dof joint in the pr2 model is the planar joint between /odom
    // and base_footprint. Here there be monsters.

    multi_dof_joint_state.header.frame_id = m_robot_model->getModelFrame();
    multi_dof_joint_state.header.seq = 0;
    multi_dof_joint_state.header.stamp = now;

    const std::vector<const moveit::core::JointModel*>& multi_dof_joints =
            m_robot_model->getMultiDOFJointModels();
    for (size_t jind = 0; jind < multi_dof_joints.size(); ++jind) {
        const moveit::core::JointModel* jm = multi_dof_joints[jind];

        multi_dof_joint_state.joint_names.push_back(jm->getName());

        const Eigen::Affine3d& joint_transform = 
                m_robot_state->getJointTransform(jm);

        geometry_msgs::Transform trans;
        tf::transformEigenToMsg(joint_transform, trans);

        multi_dof_joint_state.transforms.push_back(trans);
    }

    // TODO: attach nailgun
    req.start_state.attached_collision_objects.clear();

    req.start_state.is_diff = false;
    return true;
}

bool MoveArmCommandModel::fillGoalConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    moveit_msgs::Constraints goal_constraints;
    goal_constraints.name = "goal_constraints";

    const moveit::core::JointModelGroup* jmg =
            m_robot_model->getJointModelGroup(group_name);
    if (!jmg->isChain()) {
        ROS_INFO("Planning for joint groups that are not kinematic chains is not supported");
        return false;
    }

    auto solver = jmg->getSolverInstance();
    const std::string& tip_link = solver->getTipFrames().front();
    ROS_INFO("Planning for pose of tip link '%s' of kinematic chain", tip_link.c_str());

    const Eigen::Affine3d& T_model_tip = m_robot_state->getGlobalLinkTransform(tip_link);
    geometry_msgs::Pose tip_link_pose;
    tf::poseEigenToMsg(T_model_tip, tip_link_pose);

    // Position constraint on the tip link

    moveit_msgs::PositionConstraint goal_pos_constraint;

    goal_pos_constraint.header.frame_id = m_robot_model->getModelFrame();
    goal_pos_constraint.header.seq = 0;
    goal_pos_constraint.header.stamp = now;

    goal_pos_constraint.link_name = tip_link;

    goal_pos_constraint.target_point_offset.x = 0.0;
    goal_pos_constraint.target_point_offset.y = 0.0;
    goal_pos_constraint.target_point_offset.z = 0.0;

    // specify region within 5cm of the goal position
    shape_msgs::SolidPrimitive tolerance_volume;
    tolerance_volume.type = shape_msgs::SolidPrimitive::SPHERE;
    tolerance_volume.dimensions = { 0.05 };
    goal_pos_constraint.constraint_region.primitives.push_back(tolerance_volume);
    goal_pos_constraint.constraint_region.primitive_poses.push_back(tip_link_pose);

    goal_pos_constraint.weight = 1.0;

    // Orientation constraint on the tip link

    // specify goal orientation within 5 degrees of the goal orientation
    moveit_msgs::OrientationConstraint goal_rot_constraint;

    goal_rot_constraint.header.frame_id = m_robot_model->getModelFrame();
    goal_rot_constraint.header.seq = 0;
    goal_rot_constraint.header.stamp = now;

    goal_rot_constraint.orientation = tip_link_pose.orientation;

    goal_rot_constraint.link_name = tip_link;

    goal_rot_constraint.absolute_x_axis_tolerance = sbpl::utils::ToRadians(10.0);
    goal_rot_constraint.absolute_y_axis_tolerance = sbpl::utils::ToRadians(10.0);
    goal_rot_constraint.absolute_z_axis_tolerance = sbpl::utils::ToRadians(10.0);

    goal_rot_constraint.weight = 1.0;

//    goal_constraints.joint_constraints;
    goal_constraints.position_constraints.push_back(goal_pos_constraint);
    goal_constraints.orientation_constraints.push_back(goal_rot_constraint);
//    goal_constraints.visibility_constraints;

    req.goal_constraints.push_back(goal_constraints);

    return true;
}

bool MoveArmCommandModel::fillPathConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    return true;
}

bool MoveArmCommandModel::fillTrajectoryConstraints(
    const ros::Time& now,
    const std::string& group_name,
    moveit_msgs::MotionPlanRequest& req) const
{
    return true;
}

moveit_msgs::CollisionObject
MoveArmCommandModel::createTableCollisionObject() const
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.seq = 0;
    collision_object.header.stamp = ros::Time::now();
    collision_object.header.frame_id = "odom_combined";
    collision_object.id = m_table_name;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = m_table_size_x;
    primitive.dimensions[1] = m_table_size_y;
    primitive.dimensions[2] = m_table_size_z;

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = table_pose.orientation.y = table_pose.orientation.z = 0.0;
    table_pose.position.x = m_table_origin_x;
    table_pose.position.y = m_table_origin_y;
    table_pose.position.z = m_table_origin_z;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);

    collision_object.operation = moveit_msgs::CollisionObject::ADD;
    return collision_object;
}
