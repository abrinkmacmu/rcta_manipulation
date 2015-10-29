#include "MoveItCommandPanel.h"

#include "MoveArmCommandModel.h"

#include <sbpl_geometry_utils/utils.h>
#include <visualization_msgs/MarkerArray.h>

#include "JointVariableCommandWidget.h"

MoveItCommandPanel::MoveItCommandPanel(QWidget* parent) :
    rviz::Panel(parent),
    m_nh(),
    m_model(new MoveArmCommandModel),
    m_robot_description_line_edit(nullptr),
    m_load_robot_button(nullptr),
    m_joint_groups_combo_box(nullptr),
    m_arm_commands_group(nullptr),
    m_marker_pub(),
    m_var_cmd_widget(nullptr),
    m_table_x_spinbox(nullptr),
    m_table_y_spinbox(nullptr),
    m_table_z_spinbox(nullptr)
{
    setupGUI();

    // wait for a robot model to be loaded or for the robot's state to change
    connect(m_model.get(), SIGNAL(robotLoaded()),
            this, SLOT(updateRobot()));
    connect(m_model.get(), SIGNAL(robotStateChanged()),
            this, SLOT(syncRobot()));

    m_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_marker_array", 5);
}

MoveItCommandPanel::~MoveItCommandPanel()
{
}

void MoveItCommandPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);

    ROS_INFO("Loading config for '%s'", this->getName().toStdString().c_str());

    QString robot_description;
    config.mapGetString("robot_description", &robot_description);

    ROS_INFO("Robot Description: %s", robot_description.toStdString().c_str());

    if (m_model->loadRobot(robot_description.toStdString())) {
        m_robot_description_line_edit->setText(robot_description);
    }
}

void MoveItCommandPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);

    ROS_INFO("Saving config for '%s'", this->getName().toStdString().c_str());

    config.mapSetValue(
            "robot_description",
            QString::fromStdString(m_model->robotDescription()));

    // TODO: save the state of the MoveArmCommandModel
}

void MoveItCommandPanel::loadRobot()
{
    std::string user_robot_description =
            m_robot_description_line_edit->text().toStdString();

    if (user_robot_description.empty()) {
        QMessageBox::information(
                this,
                tr("Robot Description"),
                tr("Please enter a valid ROS parameter for the URDF"));
        return;
    }

    if (!m_model->loadRobot(user_robot_description)) {
        QMessageBox::warning(
                this,
                tr("Robot Description"),
                tr("Failed to load robot from robot description to '%1'")
                        .arg(QString::fromStdString(user_robot_description)));
    }
}

void MoveItCommandPanel::updateRobot()
{
    setupRobotGUI();
    syncRobot();
}

void MoveItCommandPanel::syncRobot()
{
    syncSpinBoxes();
    updateRobotVisualization();
}

void MoveItCommandPanel::setupGUI()
{
    ROS_INFO("Setting up the baseline GUI");

    QVBoxLayout* main_layout = new QVBoxLayout;

    // general settings
    QGroupBox* general_settings_group = new QGroupBox(tr("General Settings"));
    QVBoxLayout* general_settings_layout = new QVBoxLayout;
    QLabel* robot_description_label = new QLabel(tr("Robot Description:"));

    QHBoxLayout* robot_description_layout = new QHBoxLayout;
    m_robot_description_line_edit = new QLineEdit;
    m_load_robot_button = new QPushButton(tr("Load Robot"));
    robot_description_layout->addWidget(m_robot_description_line_edit);
    robot_description_layout->addWidget(m_load_robot_button);

    general_settings_layout->addWidget(robot_description_label);
    general_settings_layout->addLayout(robot_description_layout);
    general_settings_group->setLayout(general_settings_layout);

    main_layout->addWidget(general_settings_group);
    setLayout(main_layout);

    connect(m_load_robot_button, SIGNAL(clicked()), this, SLOT(loadRobot()));

    if (m_model->isRobotLoaded()) {
        setupRobotGUI();
    }

//    main_layout->addStretch();
}

void MoveItCommandPanel::setupRobotGUI()
{
    ROS_INFO("Setting up the Robot GUI");

    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();

    // add all joint groups as items in a combobox
    m_joint_groups_combo_box = new QComboBox;
    // set up combobox for choosing joint group to modify
    for (size_t jgind = 0;
        jgind < robot_model->getJointModelGroupNames().size();
        ++jgind)
    {
        const std::string& jg_name =
                robot_model->getJointModelGroupNames()[jgind];
        m_joint_groups_combo_box->addItem(QString::fromStdString(jg_name));
    }

    // NOTE: the first item added to the combobox will become the value of the
    // combobox

    connect(m_joint_groups_combo_box,
            SIGNAL(currentIndexChanged(const QString&)),
            this,
            SLOT(setJointGroup(const QString&)));

    m_var_cmd_widget = setupJointVariableCommandWidget();
    for (QDoubleSpinBox* spinbox : m_var_cmd_widget->spinboxes()) {
        connect(spinbox, SIGNAL(valueChanged(double)),
                this, SLOT(setJointVariableFromSpinBox(double)));
    }

    updateJointVariableCommandWidget(
        m_joint_groups_combo_box->currentText().toStdString());

    m_plan_to_position_button = new QPushButton(tr("Plan to Position"));
    connect(m_plan_to_position_button, SIGNAL(clicked()),
            this, SLOT(planToPosition()));

    m_copy_current_state_button = new QPushButton(tr("Copy Current State"));
    connect(m_copy_current_state_button, SIGNAL(clicked()),
            this, SLOT(copyCurrentState()));

    m_table_x_spinbox = new QDoubleSpinBox;
    m_table_x_spinbox->setMinimum(-10.0);
    m_table_x_spinbox->setMaximum( 10.0);
    m_table_x_spinbox->setSingleStep(0.10);
    m_table_x_spinbox->setWrapping(false);
    connect(m_table_x_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setTableX(double)));

    m_table_y_spinbox = new QDoubleSpinBox;
    m_table_y_spinbox->setMinimum(-10.0);
    m_table_y_spinbox->setMaximum( 10.0);
    m_table_y_spinbox->setSingleStep(0.10);
    m_table_y_spinbox->setWrapping(false);
    connect(m_table_y_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setTableY(double)));

    m_table_z_spinbox = new QDoubleSpinBox;
    m_table_z_spinbox->setMinimum(-10.0);
    m_table_z_spinbox->setMaximum( 10.0);
    m_table_z_spinbox->setSingleStep(0.10);
    m_table_z_spinbox->setWrapping(false);
    connect(m_table_z_spinbox, SIGNAL(valueChanged(double)),
            this, SLOT(setTableZ(double)));

    QVBoxLayout* vlayout = qobject_cast<QVBoxLayout*>(layout());
    vlayout->insertWidget(vlayout->count(), m_joint_groups_combo_box);
    vlayout->insertWidget(vlayout->count(), m_var_cmd_widget);
    vlayout->insertWidget(vlayout->count(), m_plan_to_position_button);
    vlayout->insertWidget(vlayout->count(), m_copy_current_state_button);
    vlayout->insertWidget(vlayout->count(), m_table_x_spinbox);
    vlayout->insertWidget(vlayout->count(), m_table_y_spinbox);
    vlayout->insertWidget(vlayout->count(), m_table_z_spinbox);
    vlayout->addStretch();
}

JointVariableCommandWidget*
MoveItCommandPanel::setupJointVariableCommandWidget()
{
    return new JointVariableCommandWidget(m_model.get());
}

void MoveItCommandPanel::updateJointVariableCommandWidget(
    const std::string& joint_group_name)
{
    m_var_cmd_widget->displayJointGroupCommands(joint_group_name);
}

void MoveItCommandPanel::syncSpinBoxes()
{
    if (!m_model->isRobotLoaded()) {
        ROS_WARN("Robot not yet loaded");
        return;
    }

    auto robot_model = m_model->robotModel();
    auto robot_state = m_model->robotState();

    for (int i = 0; i < (int)robot_model->getVariableCount(); ++i) {
        QDoubleSpinBox* spinbox = m_var_cmd_widget->variableIndexToSpinBox(i);

        if (isVariableAngle(i)) {
            double value =
                    sbpl::utils::ToDegrees(robot_state->getVariablePosition(i));
            if (value != spinbox->value()) {
                spinbox->setValue(value);
            }
        }
        else {
            double value = robot_state->getVariablePosition(i);
            // this check is required because the internal value of the spinbox
            // may differ from the displayed value. Apparently, scrolling the
            // spinbox by a step less than the precision will update the
            // internal value, but calling setValue will ensure that the
            // internal value is the same as the value displayed. The absence
            // of this check can result in not being able to update a joint
            // variable
            if (value != spinbox->value()) {
                spinbox->setValue(value);
            }
        }
    }
}

void MoveItCommandPanel::updateRobotVisualization()
{
    ROS_DEBUG("Updating robot visualization");

    double weight_lbs = 10.0;
    double kg_per_lb = 1.0 / 2.2;
    double gravity_z = -9.8;
    std::map<std::string, double> torques;
    torques = m_model->getRightArmTorques(
            0.0, 0.0, gravity_z * weight_lbs * kg_per_lb,
            0.0, 0.0, 0.0);

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

    // TODO: derive from the URDF
    const std::map<std::string, double> tlimits = 
    {
        { rarm_joint_names[0], 30.0 },
        { rarm_joint_names[1], 30.0 },
        { rarm_joint_names[2], 30.0 },
        { rarm_joint_names[3], 30.0 },
        { rarm_joint_names[4], 30.0 },
        { rarm_joint_names[5], 10.0 },
        { rarm_joint_names[6], 10.0 },
    };

    std::map<std::string, double> alphas;
    for (const std::string& joint_name : rarm_joint_names) {
        alphas.insert(std::make_pair(
                joint_name,
                fabs(torques.at(joint_name)) / tlimits.at(joint_name)));
    }

    if (!m_model->isRobotLoaded()) {
        ROS_WARN("Robot not yet loaded");
        return;
    }

    moveit::core::RobotModelConstPtr robot_model = m_model->robotModel();
    moveit::core::RobotStateConstPtr robot_state = m_model->robotState();

    visualization_msgs::MarkerArray marr;
    robot_state->getRobotMarkers(marr, robot_model->getLinkModelNames());

    const std::string ns =
            robot_model->getName() + std::string("_phantom");
    int id = 0;
    for (auto& marker : marr.markers) {
        float colormod = 0.0;
        bool rarm_link = false;
        // find the link associated with the given mesh filename
        for (const moveit::core::LinkModel* lm : robot_model->getLinkModels()) {
            if (marker.mesh_resource == lm->getVisualMeshFilename()) {
//                ROS_INFO("Found match for %s: %s", marker.mesh_resource.c_str(), lm->getName().c_str());
                // get the parent joint
                const moveit::core::JointModel* jm = lm->getParentJointModel();
                // check if the parent link is one of the links of the right arm
//                ROS_INFO("looking for joint '%s'", jm->getName().c_str());
                if (std::find(rarm_joint_names.begin(), rarm_joint_names.end(),
                        jm->getName()) != rarm_joint_names.end())
                {
                    // look up the color modulus
//                    ROS_INFO("Looking up torque distribution for joint %s", jm->getName().c_str());
                    colormod = (float)alphas.at(jm->getName());
                    rarm_link = true;
                    break;
                }
            }
        }

        marker.mesh_use_embedded_materials = false; //true;

        float r_base = 0.4f; // (float)100 / (float)255;
        float g_base = 0.4f; // (float)159 / (float)255;
        float b_base = 0.4f; // (float)237 / (float)255;
        if (rarm_link) {
            if (colormod > 1.0) {
                marker.color.r = marker.color.g = marker.color.b = 0.0f;
            }
            else {
                // as torque increases to limit, increase red channel and
                // decrease green and blue channels
                marker.color.r = r_base + (colormod * (1.0 - r_base));
                marker.color.g = (1.0f - colormod) * g_base;
                marker.color.b = (1.0f - colormod) * b_base;
            }
        }
        else {
            marker.color.r = r_base;
            marker.color.g = g_base;
            marker.color.b = b_base;
        }

        marker.color.a = 0.8f;
        marker.ns = ns;
        marker.id = id++;
    }

    m_marker_pub.publish(marr);
}

void MoveItCommandPanel::setJointVariableFromSpinBox(double value)
{
    QDoubleSpinBox* spinbox = qobject_cast<QDoubleSpinBox*>(sender());
    if (!spinbox) {
        ROS_WARN("setJointVariableFromSpinBox not called from a spinbox");
        return;
    }

    int vind = m_var_cmd_widget->spinboxToVariableIndex(spinbox);
    if (vind == -1) {
        ROS_ERROR("setJointVariableFromSpinBox called from spinbox not associated with a joint variable");
        return;
    }

    ROS_DEBUG("Joint variable %d set to %f from spinbox", vind, value);

    if (isVariableAngle(vind)) {
        // convert to radians and assign
        m_model->setJointVariable(vind, sbpl::utils::ToRadians(value));
    }
    else {
        // assign without conversion
        m_model->setJointVariable(vind, value);
    }
}

void MoveItCommandPanel::setJointGroup(const QString& joint_group_name)
{
    updateJointVariableCommandWidget(joint_group_name.toStdString());
}

void MoveItCommandPanel::planToPosition()
{
    std::string current_joint_group =
            m_joint_groups_combo_box->currentText().toStdString();
    m_model->planToPosition(current_joint_group);
}

void MoveItCommandPanel::copyCurrentState()
{
    m_model->copyCurrentState();
}

void MoveItCommandPanel::setTableX(double x)
{

}

void MoveItCommandPanel::setTableY(double y)
{

}

void MoveItCommandPanel::setTableZ(double z)
{

}

bool MoveItCommandPanel::isVariableAngle(int vind) const
{
    auto robot_model = m_model->robotModel();
    if (!robot_model) {
        ROS_WARN("Asking whether variable %d in uninitialized robot is an angle", vind);
        return false;
    }

    const moveit::core::JointModel* jm = robot_model->getJointOfVariable(vind);

    const std::string& var_name = robot_model->getVariableNames()[vind];

    const auto& var_bounds = jm->getVariableBounds(var_name);

    return (jm->getType() == moveit::core::JointModel::REVOLUTE ||
        (
            jm->getType() == moveit::core::JointModel::PLANAR && 
            !var_bounds.position_bounded_
        ) ||
        (
            jm->getType() == moveit::core::JointModel::FLOATING &&
            !var_bounds.position_bounded_
        ));
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(MoveItCommandPanel, rviz::Panel)
