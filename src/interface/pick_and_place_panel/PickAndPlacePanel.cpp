#include <unordered_set>
#include <Eigen/Dense>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <hdt/MoveArmCommand.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include "PickAndPlacePanel.h"

namespace hdt
{

PickAndPlacePanel::PickAndPlacePanel(QWidget* parent) :
    rviz::Panel(parent),
    nh_(),
    open_database_button_(nullptr),
    open_features_button_(nullptr),
    open_kdtree_indices_button_(nullptr),
    database_fname_label_(nullptr),
    features_fname_label_(nullptr),
    kdtree_indices_fname_label_(nullptr),
    camera_frame_selection_(nullptr),
    root_frame_selection_(nullptr),
    snap_point_cloud_button_(nullptr),
    update_grasps_button_(nullptr),
    send_move_to_pregrasp_button_(nullptr),
    send_open_gripper_command_button_(nullptr),
    send_close_gripper_command_button_(nullptr),
    listener_(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), false),
    tf_sub_(),
    frame_timeout_(10.0),
    seen_frames_(),
    pending_detection_request_(false),
    last_detection_request_(),
    last_detection_result_(),
    snapshot_cloud_pub_(),
    grasp_markers_server_("grasp_markers"),
    selected_marker_(),
    move_arm_client_(),
    object_detection_client_(),
    gripper_command_client_()
{
    setup_gui();

    snapshot_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("snapshot_cloud", 1, true);

    tf_sub_ = nh_.subscribe("tf", 10, &PickAndPlacePanel::tf_callback, this);

    move_arm_client_ = nh_.serviceClient<hdt::MoveArmCommand>("move_arm_command");

    object_detection_client_.reset(new ObjectDetectionActionClient("object_detection_action", false));
    if (!object_detection_client_) {
        ROS_ERROR("Failed to instantiate Object Detection Action Client");
    }

    gripper_command_client_.reset(new GripperCommandActionClient("gripper_controller/gripper_command_action", false));
    if (!gripper_command_client_) {
        ROS_ERROR("Failed to instantiate Gripper Command Action Client");
    }

    update_gui();
}

PickAndPlacePanel::~PickAndPlacePanel()
{
}

void PickAndPlacePanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString database_fname;
    QString features_fname;
    QString kdtree_indices_fname;
    if (config.mapGetString("DatabaseFilename", &database_fname)) {
        database_fname_label_->setText(database_fname);
    }
    if (config.mapGetString("FeaturesFilename", &features_fname)) {
        features_fname_label_->setText(features_fname);
    }
    if (config.mapGetString("KDTreeIndicesFilename", &kdtree_indices_fname)) {
        kdtree_indices_fname_label_->setText(kdtree_indices_fname);
    }

    update_gui();
}

void PickAndPlacePanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("DatabaseFilename", database_fname_label_->text());
    config.mapSetValue("FeaturesFilename", features_fname_label_->text());
    config.mapSetValue("KDTreeIndicesFilename", kdtree_indices_fname_label_->text());
}

void PickAndPlacePanel::choose_database()
{
    QString database_fname = QFileDialog::getOpenFileName(this, tr("Choose Object Database"), QString(), tr("Object Databases (*.db)"));

    if (!database_fname.isEmpty()) {
        QFileInfo database_file_info(database_fname);
        database_fname_label_->setText(database_file_info.fileName());
    }
    update_gui();
}

void PickAndPlacePanel::choose_features()
{
    QString features_fname = QFileDialog::getOpenFileName(this, tr("Choose Training Features"), QString(), tr("Training Features (*.h5)"));
    if (!features_fname.isEmpty()) {
        QFileInfo features_file_info(features_fname);
        features_fname_label_->setText(features_file_info.fileName());
    }
    update_gui();
}

void PickAndPlacePanel::choose_kdtree_indices()
{
    QString kdtree_indices_fname = QFileDialog::getOpenFileName(this, tr("Choose KD-Tree Indices"), QString(), tr("KD-Tree Indices (*.idx)"));
    if (!kdtree_indices_fname.isEmpty()) {
        QFileInfo indices_file_info(kdtree_indices_fname);
        kdtree_indices_fname_label_->setText(indices_file_info.fileName());
    }
    update_gui();
}

void PickAndPlacePanel::take_snapshot()
{
    // object detection parameters
    std::string database_fname = database_fname_label_->text().toStdString();
    std::string features_fname = features_fname_label_->text().toStdString();
    std::string kdtree_indices_fname = kdtree_indices_fname_label_->text().toStdString();

    // frames used for cleaning up the point cloud and matching
    const std::string root_frame = root_frame_selection_->currentText().toStdString();
    const std::string camera_frame = camera_frame_selection_->currentText().toStdString();

    static int object_detection_seqno = 0;
    last_detection_request_.header.seq = ++object_detection_seqno;
    last_detection_request_.header.stamp = ros::Time::now();
    last_detection_request_.header.frame_id = "";
    last_detection_request_.object_database = database_fname;
    last_detection_request_.training_features = features_fname;
    last_detection_request_.training_kdtree = kdtree_indices_fname;
    last_detection_request_.camera_frame = camera_frame;
    last_detection_request_.root_frame = root_frame;

    // TODO: configure cull parameters through the GUI
    last_detection_request_.cull_snapshot = true;
    last_detection_request_.cull_from.x = 0.3;
    last_detection_request_.cull_from.y = -0.6;
    last_detection_request_.cull_from.z = -1.6;
    last_detection_request_.cull_to.x = 2.5;
    last_detection_request_.cull_to.y = 0.6;
    last_detection_request_.cull_to.z = 1.6;

    last_detection_request_.request_snapshot = true;

    ROS_INFO("Sent goal to action server object_detection_action");
    object_detection_client_->sendGoal(
            last_detection_request_, boost::bind(&PickAndPlacePanel::object_detection_result_cb, this, _1, _2));
    pending_detection_request_ = true;

    update_gui();
}

void PickAndPlacePanel::update_grasps()
{
    selected_marker_ = std::string();

    // we have a valid row, so we have a valid match.
    if (last_detection_result_->success) {
        grasp_markers_server_.clear();
        grasp_markers_server_.applyChanges(); // todo: holdoff possible?

        int pregrasp_num = 1;
        ros::Time now = ros::Time::now();
        for (const geometry_msgs::Pose pregrasp : last_detection_result_->pregrasps.poses) {
            visualization_msgs::InteractiveMarker pregrasp_marker;
            pregrasp_marker.header.seq = 0;
            pregrasp_marker.header.stamp = now;
            pregrasp_marker.header.frame_id = last_detection_request_.root_frame;
            pregrasp_marker.pose = pregrasp;
            pregrasp_marker.name = std::string("pregrasp_") + std::to_string(pregrasp_num++);
            pregrasp_marker.description = "";
            pregrasp_marker.scale = 1.0f;

            visualization_msgs::InteractiveMarkerControl select_control;
            select_control.name = "Pregrasp Selector";
            select_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
            select_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
            select_control.always_visible = true;

            geometry_msgs::Vector3 arrow_scale;
            arrow_scale.x = 0.1;
            arrow_scale.y = 0.02;
            arrow_scale.z = 0.02;

            for (const visualization_msgs::Marker& marker : create_triad_markers(arrow_scale).markers) {
                select_control.markers.push_back(marker);
            }
            // select_control.markers.push_back(create_arrow_marker(arrow_scale));
            pregrasp_marker.controls.push_back(select_control);

            ROS_INFO("Inserting marker %s", pregrasp_marker.name.c_str());
            grasp_markers_server_.insert(pregrasp_marker, std::bind(&PickAndPlacePanel::process_feedback, this, std::placeholders::_1));
        }

        grasp_markers_server_.applyChanges();
    }
}

void PickAndPlacePanel::send_move_to_pregrasp_command()
{
    if (selected_marker_.empty()) {
        ROS_WARN("Call to send_move_to_pregrasp_command without selection");
        return;
    }

    visualization_msgs::InteractiveMarker selected_grasp_marker;
    grasp_markers_server_.get(selected_marker_, selected_grasp_marker);
    grasp_markers_server_.applyChanges(); // who knows why this is here

    // rotate the pregrasp pose to cooperate with the hdt gripper frame
    Eigen::Affine3d pr2_pregrasp_pose;
    tf::poseMsgToEigen(selected_grasp_marker.pose, pr2_pregrasp_pose);
    Eigen::AngleAxisd pr2_to_hdt_gripper_correction(M_PI / 2.0, Eigen::Vector3d(1.0, 0.0, 0.0));
    Eigen::Affine3d hdt_pregrasp_pose = pr2_pregrasp_pose * pr2_to_hdt_gripper_correction;

    if (!listener_.canTransform(selected_grasp_marker.header.frame_id, "arm_mount_panel_dummy", ros::Time(0))) {
        QMessageBox::warning(this, tr("Transform Failure"), tr("Unable to transform from %1 to %2").arg("arm_mount_panel_dummy").arg(selected_grasp_marker.header.frame_id.c_str()));
        return;
    }

    // hdt grasp pose in the camera frame
    geometry_msgs::PoseStamped pregrasp_pose;
    pregrasp_pose.header = selected_grasp_marker.header;
    tf::poseEigenToMsg(hdt_pregrasp_pose, pregrasp_pose.pose);

    // mount -> gripper = mount -> camera * camera -> gripper
    geometry_msgs::PoseStamped gripper_in_mount_frame;
    try {
        listener_.transformPose("arm_mount_panel_dummy", pregrasp_pose, gripper_in_mount_frame);
    }
    catch (const tf::TransformException& ex) {
        ROS_WARN("Fuck tf");
        return;
    }

    tf::Transform mount_to_gripper = geomsgs_pose_to_tf_transform(gripper_in_mount_frame.pose);

    // gripper -> wrist
    tf::StampedTransform gripper_to_wrist;
    std::string wrist_frame = "arm_7_gripper_lift_link";
    std::string gripper_frame = "gripper_base";
    try {
        listener_.lookupTransform(gripper_frame, wrist_frame, ros::Time(0), gripper_to_wrist);
    }
    catch (const tf::TransformException& ex) {
        ROS_WARN("Fuck tf for not being able to find a fixed transform");
        return;
    }

    // mount -> wrist = mount -> gripper * gripper -> wrist
    tf::Transform mount_to_wrist = mount_to_gripper * gripper_to_wrist;

    hdt::MoveArmCommand::Request req;
    hdt::MoveArmCommand::Response res;

    req.goal_pose = tf_transform_to_geomsgs_pose(mount_to_wrist);

    if (!move_arm_client_.call(req, res)) {
        ROS_ERROR("Call to %s failed", move_arm_client_.getService().c_str());
    }
    else {
        ROS_INFO("Call to %s finished", move_arm_client_.getService().c_str());
    }
}

void PickAndPlacePanel::send_open_gripper_command()
{
    ROS_INFO("Opening the gripper!");
    if (gripper_command_client_->isServerConnected()) {
        GripperCommandGoal goal_msg;
        goal_msg.command.position = 0.0854;
        gripper_command_client_->sendGoal(goal_msg, boost::bind(&PickAndPlacePanel::gripper_command_result_cb, this, _1, _2));
    }
    else {
        ROS_INFO("Gripper Command Client is not yet connected");
    }
}

void PickAndPlacePanel::send_close_gripper_command()
{
    ROS_INFO("Closing the gripper!");
    if (gripper_command_client_->isServerConnected()) {
        GripperCommandGoal goal_msg;
        goal_msg.command.position = 0.0;
        gripper_command_client_->sendGoal(goal_msg, boost::bind(&PickAndPlacePanel::gripper_command_result_cb, this, _1, _2));
    }
    else {
        ROS_INFO("Gripper Command Client is not yet connected");
    }
}

void PickAndPlacePanel::camera_frame_box_current_index_changed(int index)
{
    ROS_INFO("Camera Frame Box Current Index Changed!");
    update_gui();
}

void PickAndPlacePanel::camera_frame_box_edit_text_changed(const QString& text)
{
    ROS_INFO("Camera Frame Box Edit Text Changed!");
    update_gui();
}

void PickAndPlacePanel::root_frame_box_current_index_changed(int index)
{
    ROS_INFO("Root Frame Box Current Index Changed!");
    update_gui();
}

void PickAndPlacePanel::root_frame_box_edit_text_changed(const QString& text)
{
    ROS_INFO("Root Frame Box Edit Text Changed!");
    update_gui();
}

void PickAndPlacePanel::tf_callback(const tf::tfMessage::ConstPtr& msg)
{
    bool changed = false;
    for (const geometry_msgs::TransformStamped& transform : msg->transforms) {
        // check if we have a recent frame for the parent frame
        auto it = seen_frames_.find(transform.header.frame_id);
        if (it == seen_frames_.end()) {
            ROS_INFO("Adding %s to available frames", transform.header.frame_id.c_str());
            seen_frames_.insert(std::make_pair(transform.header.frame_id, transform.header.stamp));
            changed = true;
        }
        else {
            it->second = transform.header.stamp;
        }

        // check if we have a recent frame for the child frame
        it = seen_frames_.find(transform.child_frame_id);
        if (it == seen_frames_.end()) {
            ROS_INFO("Adding %s to available frames", transform.child_frame_id.c_str());
            seen_frames_.insert(std::make_pair(transform.child_frame_id, transform.header.stamp));
            changed = true;
        }
        else {
            it->second = transform.header.stamp;
        }
    }

    // remove old frames
//    ros::Time now = ros::Time::now();
//    auto it = seen_frames_.begin();
//    while (it != seen_frames_.end()) {
//        bool erase = (it->second < now - ros::Duration(frame_timeout_));
//        if (erase) {
//            auto eit = it;
//            ++it;
//            changed = true;
//            ROS_INFO("Removing %s from available frames", eit->first.c_str());
//            seen_frames_.erase(eit);
//        }
//        else {
//            ++it;
//        }
//    }

    if (changed) {
        // update combo box options
        update_combo_box(*camera_frame_selection_, seen_frames_);
        update_combo_box(*root_frame_selection_, seen_frames_);
    }
}

void PickAndPlacePanel::update_gui()
{
    snap_point_cloud_button_->setEnabled(!database_fname_label_->text().isEmpty() &&
                                         !features_fname_label_->text().isEmpty() &&
                                         !kdtree_indices_fname_label_->text().isEmpty() &&
                                         !camera_frame_selection_->currentText().isEmpty() &&
                                         !root_frame_selection_->currentText().isEmpty() &&
                                         !pending_detection_request_);
    update_grasps_button_->setEnabled(last_detection_result_ && last_detection_result_->success);
    send_move_to_pregrasp_button_->setEnabled(!selected_marker_.empty());
}

int PickAndPlacePanel::find_item(const QComboBox& combo_box, const std::string& item) const
{
    for (int i = 0; i < combo_box.count(); ++i) {
        if (combo_box.itemText(i).toStdString() == item) {
            return i;
        }
    }
    return -1;
}

void PickAndPlacePanel::update_combo_box(QComboBox& combo_box, const std::map<std::string, ros::Time>& entries)
{
    std::unordered_set<std::string> searched;
    for (int i = 0; i < combo_box.count(); ) {
        // remove items from the combo box that are not part of entries
        QString item_text = combo_box.itemText(i);
        if (entries.find(item_text.toStdString()) == entries.end()) {
            combo_box.removeItem(i);
        }
        else {
            ++i;
        }
        searched.insert(item_text.toStdString());
    }

    // add new elements from entries (elements that were checked in the entry pass above but were not removed)
    for (const auto& entry : entries) {
        if (searched.find(entry.first) == searched.end()) {
            combo_box.addItem(QString::fromStdString(entry.first));
        }
    }
}

void PickAndPlacePanel::process_feedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback_msg)
{
    print_interactive_marker_feedback(*feedback_msg);

    if (feedback_msg->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN &&
        selected_marker_ != feedback_msg->marker_name)
    {
        grasp_markers_server_.applyChanges();

        if (!selected_marker_.empty()) {
            // unselect marker
            visualization_msgs::InteractiveMarker previous_selection;
            grasp_markers_server_.get(selected_marker_, previous_selection);
            ROS_INFO("Previous selection was in frame %s", previous_selection.header.frame_id.c_str());
            ROS_INFO("Previous selection was at pose: x: %0.3f, y: %0.3f, z: %0.3f", previous_selection.pose.position.x, previous_selection.pose.position.y, previous_selection.pose.position.z);
            previous_selection.controls.front().markers.front().color.r = 0.0;
            grasp_markers_server_.insert(previous_selection);
            grasp_markers_server_.applyChanges();
        }

        // select this marker
        selected_marker_ = feedback_msg->marker_name;
        visualization_msgs::InteractiveMarker new_selection;
        grasp_markers_server_.get(selected_marker_, new_selection);
        ROS_INFO("New selection was in frame %s", new_selection.header.frame_id.c_str());
        ROS_INFO("New selection was at pose: x: %0.3f, y: %0.3f, z: %0.3f", new_selection.pose.position.x, new_selection.pose.position.y, new_selection.pose.position.z);
        new_selection.controls.front().markers.front().color.r = 1.0;
        grasp_markers_server_.insert(new_selection);

        grasp_markers_server_.applyChanges();
    }

    update_gui();
}

void PickAndPlacePanel::print_interactive_marker_feedback(const visualization_msgs::InteractiveMarkerFeedback& feedback_msg) const
{
    ROS_INFO("Processing feedback for interactive marker %s", feedback_msg.marker_name.c_str());
    ROS_INFO("  header:");
    ROS_INFO("    seq: %u", feedback_msg.header.seq);
    ROS_INFO("    stamp: %s", boost::posix_time::to_simple_string(feedback_msg.header.stamp.toBoost()).c_str());
    ROS_INFO("    frame_id: %s", feedback_msg.header.frame_id.c_str());
    ROS_INFO("  client_id: %s", feedback_msg.client_id.c_str());
    ROS_INFO("  marker_name: %s", feedback_msg.marker_name.c_str());
    ROS_INFO("  control_name: %s", feedback_msg.control_name.c_str());
    ROS_INFO("  event_type: %d", (int)feedback_msg.event_type);
    ROS_INFO("  pose:");
    ROS_INFO("    position:");
    ROS_INFO("      x: %0.3f", feedback_msg.pose.position.x);
    ROS_INFO("      y: %0.3f", feedback_msg.pose.position.y);
    ROS_INFO("      z: %0.3f", feedback_msg.pose.position.z);
    ROS_INFO("    orientation:");
    ROS_INFO("      w: %0.3f", feedback_msg.pose.orientation.w);
    ROS_INFO("      x: %0.3f", feedback_msg.pose.orientation.x);
    ROS_INFO("      y: %0.3f", feedback_msg.pose.orientation.y);
    ROS_INFO("      z: %0.3f", feedback_msg.pose.orientation.z);
    ROS_INFO("  menu_entry_id: %u", feedback_msg.menu_entry_id);
    ROS_INFO("  mouse_point:");
    ROS_INFO("    x:", feedback_msg.mouse_point.x);
    ROS_INFO("    y:", feedback_msg.mouse_point.y);
    ROS_INFO("    z:", feedback_msg.mouse_point.z);
    ROS_INFO("  mouse_point_valid: %s", feedback_msg.mouse_point_valid ? "true" : "false");
}

visualization_msgs::Marker PickAndPlacePanel::create_arrow_marker(const geometry_msgs::Vector3 &scale)
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

visualization_msgs::MarkerArray PickAndPlacePanel::create_triad_markers(const geometry_msgs::Vector3& scale)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker m = create_arrow_marker(scale);
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.7;
    markers.markers.push_back(m);

    Eigen::Quaterniond q(1.0, 0.0, 0.0, 0.0);

    Eigen::AngleAxisd rotate_z(M_PI / 2.0, Eigen::Vector3d(0.0, 0.0, 1.0));

    q = rotate_z * q;
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();

    m.color.r = 0.0;
    m.color.g = 1.0;
    markers.markers.push_back(m);

    Eigen::AngleAxisd rotate_y(-M_PI / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0));
    q = rotate_y * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();

    m.color.g = 0.0;
    m.color.b = 1.0;
    markers.markers.push_back(m);

    return markers;
}

void PickAndPlacePanel::gripper_command_active_cb()
{

}

void PickAndPlacePanel::gripper_command_feedback_cb(const GripperCommandFeedback::ConstPtr& feedback)
{

}

void PickAndPlacePanel::gripper_command_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const GripperCommandResult::ConstPtr& result)
{
    ROS_INFO("Gripper command completed");
}

void PickAndPlacePanel::object_detection_active_cb()
{
    update_gui();
}

void PickAndPlacePanel::object_detection_feedback_cb(const hdt::ObjectDetectionFeedback::ConstPtr& feedback)
{
}

void PickAndPlacePanel::object_detection_result_cb(
    const actionlib::SimpleClientGoalState& state,
    const hdt::ObjectDetectionResult::ConstPtr& result)
{
    ROS_INFO("Received object detection result");
    if (!result) {
        ROS_WARN("Object Detection Result is null");
        return;
    }

    if (last_detection_request_.request_snapshot) {
        snapshot_cloud_pub_.publish(result->snapshot_cloud);
    }

    if (result->success) {
        ROS_INFO("A successful match was found");
        if (result->match_score != result->match_score) {
             QMessageBox::warning(this, tr("Match Results"), tr("Match succeeded but detector was unable to score it"));
        }
        else {
             QMessageBox::information(this, tr("Match Results"), tr("Match succeeded with score %1").arg(result->match_score));
        }
    }
    else {
        ROS_WARN("No matches found!");
    }

    last_detection_result_ = result;
    pending_detection_request_ = false;

    update_gui();
}

void PickAndPlacePanel::setup_gui()
{
    QVBoxLayout* main_layout = new QVBoxLayout;

    QLabel* training_data_label = new QLabel(tr("Training Data"), this);
    main_layout->addWidget(training_data_label);

    QGridLayout* detection_data_layout = new QGridLayout;

    open_database_button_ = new QPushButton(tr("Choose Object Database"), this);
    open_features_button_ = new QPushButton(tr("Choose Training Data (features)"), this);
    open_kdtree_indices_button_ = new QPushButton(tr("Choose Training Data (kdtree)"), this);

    database_fname_label_ = new QLabel(this);
    features_fname_label_ = new QLabel(this);
    kdtree_indices_fname_label_ = new QLabel(this);

    detection_data_layout->addWidget(open_database_button_, 0, 0);
    detection_data_layout->addWidget(database_fname_label_, 0, 1);
    detection_data_layout->addWidget(open_features_button_, 1, 0);
    detection_data_layout->addWidget(features_fname_label_, 1, 1);
    detection_data_layout->addWidget(open_kdtree_indices_button_, 2, 0);
    detection_data_layout->addWidget(kdtree_indices_fname_label_, 2, 1);

    main_layout->addLayout(detection_data_layout);

    QLabel* frame_data_label = new QLabel(tr("Frame Selection"), this);
    main_layout->addWidget(frame_data_label);

    QHBoxLayout* frame_data_layout = new QHBoxLayout;

    QLabel* camera_frame_box_label = new QLabel(tr("Camera Frame: "), this);
    camera_frame_selection_ = new QComboBox(this);
    QLabel* root_frame_box_label = new QLabel(tr("Root Frame: "), this);
    root_frame_selection_ = new QComboBox(this);

    frame_data_layout->addWidget(camera_frame_box_label);
    frame_data_layout->addWidget(camera_frame_selection_);
    frame_data_layout->addWidget(root_frame_box_label);
    frame_data_layout->addWidget(root_frame_selection_);

    main_layout->addLayout(frame_data_layout);

    QLabel* commands_label = new QLabel(tr("Commands"), this);
    main_layout->addWidget(commands_label);

    snap_point_cloud_button_ = new QPushButton("Take Snapshot");
    main_layout->addWidget(snap_point_cloud_button_);

    update_grasps_button_ = new QPushButton("Update Grasps");
    main_layout->addWidget(update_grasps_button_);

    send_move_to_pregrasp_button_ = new QPushButton("Move to Pre-Grasp");
    main_layout->addWidget(send_move_to_pregrasp_button_);

    send_open_gripper_command_button_ = new QPushButton("Open Gripper");
    main_layout->addWidget(send_open_gripper_command_button_);

    send_close_gripper_command_button_ = new QPushButton("Close Gripper");
    main_layout->addWidget(send_close_gripper_command_button_);

    setLayout(main_layout);

    connect(open_database_button_, SIGNAL(clicked()), this, SLOT(choose_database()));
    connect(open_features_button_, SIGNAL(clicked()), this, SLOT(choose_features()));
    connect(open_kdtree_indices_button_, SIGNAL(clicked()), this, SLOT(choose_kdtree_indices()));

    connect(camera_frame_selection_,    SIGNAL(currentIndexChanged(int)),           this, SLOT(camera_frame_box_current_index_changed(int)));
    connect(root_frame_selection_,      SIGNAL(currentIndexChanged(int)),           this, SLOT(root_frame_box_current_index_changed(int)));
    connect(camera_frame_selection_,    SIGNAL(editTextChanged(const QString&)),    this, SLOT(camera_frame_box_edit_text_changed(const QString&)));
    connect(root_frame_selection_,      SIGNAL(editTextChanged(const QString&)),    this, SLOT(camera_frame_box_edit_text_changed(const QString&)));

    connect(snap_point_cloud_button_, SIGNAL(clicked()), this, SLOT(take_snapshot()));
    connect(update_grasps_button_, SIGNAL(clicked()), this, SLOT(update_grasps()));
    connect(send_move_to_pregrasp_button_, SIGNAL(clicked()), this, SLOT(send_move_to_pregrasp_command()));
    connect(send_open_gripper_command_button_, SIGNAL(clicked()), this, SLOT(send_open_gripper_command()));
    connect(send_close_gripper_command_button_, SIGNAL(clicked()), this, SLOT(send_close_gripper_command()));
}

tf::Transform PickAndPlacePanel::geomsgs_pose_to_tf_transform(const geometry_msgs::Pose& pose) const
{
    return tf::Transform(
            tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::Pose PickAndPlacePanel::tf_transform_to_geomsgs_pose(const tf::Transform& transform) const
{
    geometry_msgs::Pose pose;
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation.w = transform.getRotation().w();
    pose.orientation.x = transform.getRotation().x();
    pose.orientation.y = transform.getRotation().y();
    pose.orientation.z = transform.getRotation().z();
    return pose;
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::PickAndPlacePanel, rviz::Panel);
