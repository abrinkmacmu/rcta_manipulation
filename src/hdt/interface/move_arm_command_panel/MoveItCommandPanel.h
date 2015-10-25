#ifndef MoveItCommandPanel_h
#define MoveItCommandPanel_h

// standard includes
#include <map>
#include <memory>
#include <string>

// system includes
#include <QtGui>
#include <moveit/robot_model/robot_model.h>
#include <ros/ros.h>
#include <rviz/panel.h>

class MoveArmCommandModel;
class JointVariableCommandWidget;

class MoveItCommandPanel : public rviz::Panel
{
    Q_OBJECT

public:

    MoveItCommandPanel(QWidget* parent = 0);
    ~MoveItCommandPanel();

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

public Q_SLOTS:

    /// \brief Load the robot with the parameter in the Robot Description line
    ///     edit box
    void loadRobot();

    /// \brief Update the GUI and visualizations
    void updateRobot();

    void syncRobot();

    /// \brief Update the robot visualization to reflect the current state of
    ///     the robot
    void updateRobotVisualization();

    void setJointVariableFromSpinBox(double value);
    void setJointGroup(const QString& joint_group_name);
    void planToPosition();

private:

    ros::NodeHandle m_nh;

    std::unique_ptr<MoveArmCommandModel> m_model;

    QLineEdit* m_robot_description_line_edit;
    QPushButton* m_load_robot_button;

    QComboBox* m_joint_groups_combo_box;
    QGroupBox* m_arm_commands_group;

    QPushButton* m_plan_to_position_button;

    ros::Publisher m_marker_pub;

    JointVariableCommandWidget* m_var_cmd_widget;

    /// \brief Setup the baseline GUI for loading robots from URDF parameter
    void setupGUI();

    void setupRobotGUI();

    // Create a scroll area containing spinboxes for all joint variables. Also
    // creates a bijection between joint variable indices and spinboxes and
    // automatically connects all spinboxes to the setJointVariableFromSpinbox
    // slot. Assumes a robot model has already been loaded into the model.
    JointVariableCommandWidget* setupJointVariableCommandWidget();
    void updateJointVariableCommandWidget(const std::string& joint_group_name);

    void syncSpinBoxes();

    bool isVariableAngle(int vind) const;
};

#endif
