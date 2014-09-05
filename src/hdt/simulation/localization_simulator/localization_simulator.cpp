#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <hdt/common/utils/RunUponDestruction.h>
#include <hdt/TeleportAndaliteCommandAction.h>
#include <hdt/common/msg_utils/msg_utils.h>

class LocalizationSimulator
{
public:

    LocalizationSimulator();

    enum MainResult
    {
        SUCCESS,
        FAILED_TO_INITIALIZE
    };
    int run();

private:

    bool initialize();

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::string action_name_;

    std::string world_frame_nwu_name_;
    std::string world_frame_ned_name_;
    std::string robot_frame_nwu_name_;
    std::string robot_frame_ned_name_;

    tf::TransformBroadcaster broadcaster_;

    Eigen::Affine3d world_to_robot_;

    typedef actionlib::SimpleActionServer<hdt::TeleportAndaliteCommandAction> TeleportAndaliteCommandActionServer;
    std::unique_ptr<TeleportAndaliteCommandActionServer> as_;

    void goal_callback();
    void preempt_callback();
};

LocalizationSimulator::LocalizationSimulator() :
    nh_(),
    ph_("~"),
    action_name_("teleport_andalite_command"),
    world_frame_nwu_name_("abs_ned"),
    world_frame_ned_name_("abs_nwu"),
    robot_frame_nwu_name_("base_footprint"),
    robot_frame_ned_name_("robot_ned"),
    broadcaster_(),
    world_to_robot_(Eigen::Affine3d::Identity()),
    as_()
{
}

int LocalizationSimulator::run()
{
    if (!initialize()) {
        ROS_ERROR("Failed to initialize Localization Simulator");
        return FAILED_TO_INITIALIZE;
    }

    ros::Rate loop_rate(10.0);
    while (ros::ok()) {
        RunUponDestruction rod([&]() { loop_rate.sleep(); });

        ros::spinOnce();

        ros::Time now = ros::Time::now();

        tf::Quaternion ned_to_nwu_rotation(tf::Vector3(1.0, 0.0, 0.0), M_PI);
        tf::Transform ned_to_nwu(ned_to_nwu_rotation, tf::Vector3(0.0, 0.0, 0.0));

        // publish robot coordinate frame using NED conventions for convenience
        tf::StampedTransform robot_nwu_to_ned_stamped(ned_to_nwu, now, robot_frame_nwu_name_, robot_frame_ned_name_);
        broadcaster_.sendTransform(robot_nwu_to_ned_stamped);

        // publish world coordinate frame to robot coordinate frame transform (global pose)
        tf::Transform world_ned_to_robot_ned;
        msg_utils::convert(world_to_robot_, world_ned_to_robot_ned);
        tf::StampedTransform world_ned_to_robot_ned_stamped(world_ned_to_robot_ned, now, world_frame_ned_name_, robot_frame_ned_name_);
        broadcaster_.sendTransform(robot_nwu_to_ned_stamped);

        // publish world coordinate frame using NWU conventions for convenience
        tf::StampedTransform world_ned_to_nwu_stamped(ned_to_nwu, now, world_frame_ned_name_, robot_frame_nwu_name_);
        broadcaster_.sendTransform(world_ned_to_nwu_stamped);

        // TODO: publish /laser transform?
    }

    return SUCCESS;
}

bool LocalizationSimulator::initialize()
{
    as_.reset(new TeleportAndaliteCommandActionServer(action_name_, false));
    if (!as_) {
        ROS_ERROR("Failed to instantiate Teleport Andalite Command Action Server");
        return false;
    }

    as_->registerGoalCallback(boost::bind(&LocalizationSimulator::goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&LocalizationSimulator::preempt_callback, this));

    ROS_INFO("Starting action server '%s'...", action_name_.c_str());
    as_->start();
    ROS_INFO("Action server started");

    return true;
}

void LocalizationSimulator::goal_callback()
{
    auto current_goal = as_->acceptNewGoal();
    tf::poseMsgToEigen(current_goal->global_pose, world_to_robot_);
    as_->setSucceeded();
}

void LocalizationSimulator::preempt_callback()
{
    // shouldn't ever need to do this...I mean it
}

int main(int argc, char* argv[])
{
    return LocalizationSimulator().run();
}
