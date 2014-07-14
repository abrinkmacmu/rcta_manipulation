#ifndef hdt_ManipulatorCommandPanel_h
#define hdt_ManipulatorCommandPanel_h

#include <atomic>
#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <QtGui>
#include <actionlib/client/simple_action_client.h>
#include <hdt_description/RobotModel.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <hdt/MoveArmCommandAction.h>

class QPushButton;

namespace hdt
{

class ManipulatorCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    ManipulatorCommandPanel(QWidget *parent = 0);
    ~ManipulatorCommandPanel();

public Q_SLOTS:

    void copy_current_state();
    void refresh_robot_description();
    void send_move_arm_command();
    void cycle_ik_solutions();

    void change_joint_1(int value);
    void change_joint_2(int value);
    void change_joint_3(int value);
    void change_joint_4(int value);
    void change_joint_5(int value);
    void change_joint_6(int value);
    void change_joint_7(int value);

private:

    bool initialized_;

    ros::NodeHandle nh_;

    typedef actionlib::SimpleActionClient<hdt::MoveArmCommandAction> MoveArmCommandActionClient;
    std::unique_ptr<MoveArmCommandActionClient> move_arm_client_;
    bool pending_move_arm_command_;

    hdt::RobotModel robot_model_;

    std::unique_ptr<robot_model_loader::RobotModelLoader> rm_loader_;
    robot_model::RobotModelPtr rm_;
    robot_state::RobotStatePtr rs_;

    interactive_markers::InteractiveMarkerServer server_;

    ros::Subscriber joint_states_sub_;
    ros::Publisher robot_markers_pub_;

    sensor_msgs::JointState last_joint_state_;

    std::vector<visualization_msgs::InteractiveMarker> interactive_markers_;
    std::string tip_link_;
    std::string base_link_;

    QPushButton* copy_current_state_button_;
    QPushButton* refresh_robot_desc_button_;
    QPushButton* send_move_arm_command_button_;
    QPushButton* cycle_ik_solutions_button_;
    QSlider* joint_1_slider_;
    QSlider* joint_2_slider_;
    QSlider* joint_3_slider_;
    QSlider* joint_4_slider_;
    QSlider* joint_5_slider_;
    QSlider* joint_6_slider_;
    QSlider* joint_7_slider_;

    Eigen::Affine3d root_to_first_link_;
    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    bool do_init();
    bool check_robot_model_consistency();

    void do_process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void publish_transform_network();
    void publish_joint_states();
    void publish_phantom_robot_visualizations();

    /// @brief Return the tip link of a joint model group if the joint model group has only one root
    std::string get_tip_link(const robot_model::JointModelGroup& joint_model_group) const;

    /// @brief Return the base link of a joint model group if the joint model group has only one root
    std::string get_base_link(const robot_model::JointModelGroup& joint_model_group) const;

    std::vector<visualization_msgs::InteractiveMarkerControl> create_sixdof_controls() const;

    /// @brief Get the value of a particular joint in a joint state.
    /// @return true if the joint was found; false otherwise
    bool get_joint_value(const sensor_msgs::JointState& joint_state, const std::string& joint, double& joint_value) const;

    void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);

    bool reinit_robot();

    void monitor_move_command();

    void move_arm_command_active_cb();
    void move_arm_command_feedback_cb(const hdt::MoveArmCommandFeedback::ConstPtr& feedback);
    void move_arm_command_result_cb(
        const actionlib::SimpleClientGoalState& state,
        const hdt::MoveArmCommandResult::ConstPtr& result);

    bool gatherRobotMarkers(
        const robot_state::RobotState& robot_state,
        const std::vector<std::string>& link_names,
        const std_msgs::ColorRGBA& color,
        const std::string& ns,
        const ros::Duration& d,
        visualization_msgs::MarkerArray& markers,
        bool include_attached = false);

    static void joint_state_to_joint_array(const sensor_msgs::JointState&);

    std::vector<double> get_current_joint_angles() const;
    std::vector<double> get_phantom_joint_angles() const;

    bool set_phantom_joint_angles(const std::vector<double>& joint_angles);

    void update_sliders();
    void update_gui();
};

} // namespace hdt

#endif
