#include <unordered_set>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <hdt/MoveArmCommand.h>
#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include "PickAndPlacePanel.h"

namespace hdt
{

PickAndPlacePanel::PickAndPlacePanel(QWidget* parent) :
    rviz::Panel(parent),
    listener_(ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME, false)), // no calling waitForTransform on this guy
    camera_frame_selection_(nullptr),
    root_frame_selection_(nullptr),
    frame_timeout_(10.0),
    grasp_markers_server_("grasp_markers"),
    gripper_command_client_()
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

    point_cloud_topic_ = "camera/depth_registered/points";
    point_cloud_sub_ = nh_.subscribe(point_cloud_topic_, 1, &PickAndPlacePanel::point_cloud_callback, this);
    // point_cloud_sub_.subscribe(nh_, point_cloud_topic);

    snapshot_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("snapshot_cloud", 1, true);

    connect(open_database_button_, SIGNAL(clicked()), this, SLOT(choose_database()));
    connect(open_features_button_, SIGNAL(clicked()), this, SLOT(choose_features()));
    connect(open_kdtree_indices_button_, SIGNAL(clicked()), this, SLOT(choose_kdtree_indices()));

   // connect(camera_frame_selection_, SIGNAL(activated(int)),                        this, SLOT(camera_frame_box_activated(int)));
//    connect(camera_frame_selection_, SIGNAL(activated(const QString&)),             this, SLOT(camera_frame_box_activated(const QString&)));
   connect(camera_frame_selection_, SIGNAL(currentIndexChanged(int)),              this, SLOT(camera_frame_box_current_index_changed(int)));
//    connect(camera_frame_selection_, SIGNAL(currentIndexChanged(const QString&)),   this, SLOT(camera_frame_box_current_index_changed(const QString&)));
//    connect(camera_frame_selection_, SIGNAL(editTextChanged(const QString&)),       this, SLOT(camera_frame_box_edit_text_changed(const QString&)));
//    connect(camera_frame_selection_, SIGNAL(highlighted(int)),                      this, SLOT(camera_frame_box_highlighted(int)));
//    connect(camera_frame_selection_, SIGNAL(highlighted(const QString&)),           this, SLOT(camera_frame_box_highlighted(const QString&)));

    connect(snap_point_cloud_button_, SIGNAL(clicked()), this, SLOT(take_snapshot()));
    connect(update_grasps_button_, SIGNAL(clicked()), this, SLOT(update_grasps()));
    connect(send_move_to_pregrasp_button_, SIGNAL(clicked()), this, SLOT(send_move_to_pregrasp_command()));
    connect(send_open_gripper_command_button_, SIGNAL(clicked()), this, SLOT(send_open_gripper_command()));
    connect(send_close_gripper_command_button_, SIGNAL(clicked()), this, SLOT(send_close_gripper_command()));

    tf_sub_ = nh_.subscribe("tf", 10, &PickAndPlacePanel::tf_callback, this);

    move_arm_client_ = nh_.serviceClient<hdt::MoveArmCommand>("move_arm_command");

    gripper_command_client_.reset(new GripperCommandActionClient("gripper_controller/gripper_command_action"));
    if (!gripper_command_client_) {
        ROS_ERROR("Failed to instantiate Gripper Command Action Client");
    }
    else {

    }

    update_gui();
}

PickAndPlacePanel::~PickAndPlacePanel()
{
}

void PickAndPlacePanel::load(const rviz::Config& config)
{
    // todo: implement
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
        database_fname_label_->setText(database_fname);
    }
    update_gui();
}

void PickAndPlacePanel::choose_features()
{
    QString features_fname = QFileDialog::getOpenFileName(this, tr("Choose Training Features"), QString(), tr("Training Features (*.h5)"));
    if (!features_fname.isEmpty()) {
        features_fname_label_->setText(features_fname);
    }
    update_gui();
}

void PickAndPlacePanel::choose_kdtree_indices()
{
    QString kdtree_indices_fname = QFileDialog::getOpenFileName(this, tr("Choose KD-Tree Indices"), QString(), tr("KD-Tree Indices (*.idx)"));
    if (!kdtree_indices_fname.isEmpty()) {
        kdtree_indices_fname_label_->setText(kdtree_indices_fname);
    }
    update_gui();
}

void PickAndPlacePanel::take_snapshot()
{
    snapshot_cloud_ = last_cloud_msg_;

    if (!snapshot_cloud_) {
        QMessageBox::warning(this, tr("Invalid Point Cloud"), tr("Pick and Place Plugin has not yet received a point cloud on %1").arg(point_cloud_topic_.c_str()));
        return;
    }

    const std::string& cloud_frame = snapshot_cloud_->header.frame_id;
    const std::string root_frame = root_frame_selection_->currentText().toStdString();
    const std::string camera_frame = camera_frame_selection_->currentText().toStdString();

    ObjectFinder::CullingOptions cull_options;
    cull_options.cull = true;
    cull_options.from.x = 0.3;
    cull_options.from.y = -0.6;
    cull_options.from.z = -1.6;
    cull_options.to.x = 2.5;
    cull_options.to.y = 0.6;
    cull_options.to.z = 1.6;

    sensor_msgs::PointCloud2 transformed_snapshot;
    pcl_ros::transformPointCloud(root_frame, *snapshot_cloud_, transformed_snapshot, listener_);
    snapshot_cloud_pub_.publish(object_detector_->cull_point_cloud_msg(transformed_snapshot, cull_options.from, cull_options.to));

    ros::spinOnce(); // show point cloud before detection

    ros::Duration(1.0).sleep();

    ROS_INFO("Smile! cloud frame: %s, root frame: %s, camera frame: %s", cloud_frame.c_str(), root_frame.c_str(), camera_frame.c_str());

    bool transforms_available = true;
    transforms_available &= listener_.canTransform(camera_frame, cloud_frame, ros::Time(0));
    transforms_available &= listener_.canTransform(root_frame, camera_frame, ros::Time(0));
    if (!transforms_available) {
        QMessageBox::warning(this, tr("Transforms Not Available"), tr("Transforms not available between cloud, camera, and root frames"));
        return;
    }

    std::string database_fname = database_fname_label_->text().toStdString();
    std::string features_fname = features_fname_label_->text().toStdString();
    std::string kdtree_indices_fname = kdtree_indices_fname_label_->text().toStdString();

    // lazily (re)initialize the object detector when a snapshot is requested with difference settings
    if (!object_detector_ ||
        database_fname != last_snapshot_database_ ||
        features_fname != last_snapshot_features_ ||
        kdtree_indices_fname != last_snapshot_kdtree_indices_)
    {
        ROS_INFO("Engaging Detector! object database: %s, features: %s, kdtree indices: %s", database_fname.c_str(), features_fname.c_str(), kdtree_indices_fname.c_str());

        ObjectFinder::VisualizationOptions visopts;
        visopts.visualize_culled_cloud = false;
        visopts.visualize_best_cluster = true;
        visopts.visualize_best_match = true;
        visopts.visualize_segmented_plane = true;
        visopts.visualize_all_clusters = true;
        visopts.visualize_fixed_match = true;

        std::unique_ptr<ObjectFinder> object_detector(new ObjectFinder(visopts));

        if (!object_detector->initialize(database_fname, features_fname, kdtree_indices_fname)) {
            QMessageBox::warning(this, tr("Initialization Failure"), tr("Failed to initialize Object Detector"));
            return;
        }

        object_detector_ = std::move(object_detector);
        last_snapshot_database_ = std::move(database_fname);
        last_snapshot_features_ = std::move(features_fname);
        last_snapshot_kdtree_indices_ = std::move(kdtree_indices_fname);
    }

    assert(object_detector_);

    last_detection_query_.snapshot = snapshot_cloud_;
    last_detection_query_.camera_frame = camera_frame;
    last_detection_query_.root_frame = root_frame;
    last_detection_query_.cull_options = cull_options;

    last_match_ = object_detector_->match(*snapshot_cloud_, camera_frame, root_frame, listener_, cull_options);
    bool success = last_match_.valid();

    if (success) {
        assert(last_match_.row > 0);
        ROS_INFO("A successful match was found");
        object_detector_->get_grasps(last_match_,
                                     camera_frame,
                                     root_frame,
                                     last_detection_query_.grasps,
                                     last_detection_query_.pregrasps,
                                     listener_);

        double match_score = object_detector_->score_match(last_match_, camera_frame);
        if (match_score != match_score) { // nan check
            QMessageBox::information(this, tr("Match Results"), tr("Match succeeded with score %1").arg(match_score));
        }
        else {
            QMessageBox::warning(this, tr("Match Results"), tr("Match succeeded but detector was unable to score it"));
        }
    }
    else {
        ROS_WARN("No matches found! :(");
    }
}

void PickAndPlacePanel::update_grasps()
{
    selected_marker_ = std::string();

    // we have a valid row, so we have a valid match.
    if (last_match_.row != 0) {
        grasp_markers_server_.clear();
        grasp_markers_server_.applyChanges(); // todo: holdoff possible?

        int pregrasp_num = 1;
        ros::Time now = ros::Time::now();
        for (const geometry_msgs::Pose pregrasp : last_detection_query_.pregrasps.poses) {
            visualization_msgs::InteractiveMarker pregrasp_marker;
            pregrasp_marker.header.seq = 0;
            pregrasp_marker.header.stamp = now;
            pregrasp_marker.header.frame_id = last_detection_query_.root_frame;
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
            select_control.markers.push_back(create_arrow_marker(arrow_scale));
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

    grasp_markers_server_.applyChanges();
    if (!listener_.canTransform(selected_grasp_marker.header.frame_id, "arm_mount_panel_dummy", ros::Time(0))) {
        QMessageBox::warning(this, tr("Transform Failure"), tr("Unable to transform from %1 to %2").arg("arm_mount_panel_dummy").arg(selected_grasp_marker.header.frame_id.c_str()));
        return;
    }

    tf::StampedTransform mount_to_marker_frame;

    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header = selected_grasp_marker.header;
    grasp_pose.pose = selected_grasp_marker.pose;

    geometry_msgs::PoseStamped grasp_pose_in_mount_frame;
    try {
        listener_.transformPose("arm_mount_panel_dummy", grasp_pose, grasp_pose_in_mount_frame);
    }
    catch (const tf::TransformException& ex) {
        ROS_WARN("Fuck tf");
        return;
    }

    hdt::MoveArmCommand::Request req;
    hdt::MoveArmCommand::Response res;

    req.goal_pose = grasp_pose_in_mount_frame.pose;

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
        GoalHandle goal = gripper_command_client_->sendGoal(goal_msg,
                boost::bind(&PickAndPlacePanel::gripper_command_action_transition, this, _1),
                boost::bind(&PickAndPlacePanel::gripper_command_action_feedback, this, _1, _2));
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
        GoalHandle goal = gripper_command_client_->sendGoal(goal_msg,
                boost::bind(&PickAndPlacePanel::gripper_command_action_transition, this, _1),
                boost::bind(&PickAndPlacePanel::gripper_command_action_feedback, this, _1, _2));
    }
    else {
        ROS_INFO("Gripper Command Client is not yet connected");
    }
}

void PickAndPlacePanel::camera_frame_box_activated(int index)
{
    ROS_INFO("Camera Frame Box Activated!");
}

void PickAndPlacePanel::camera_frame_box_activated(const QString& text)
{
    ROS_INFO("Camera Frame Box Activated!");
}

void PickAndPlacePanel::camera_frame_box_current_index_changed(int index)
{
    // ROS_INFO("Camera Frame Box Current Index Changed to %d!", index);
    update_gui();
}

void PickAndPlacePanel::camera_frame_box_current_index_changed(const QString& text)
{
    // ROS_INFO("Camera Frame Box Current Index Changed to %s!", text.toStdString().c_str());
}

void PickAndPlacePanel::camera_frame_box_edit_text_changed(const QString& text)
{
    ROS_INFO("Camera Frame Box Edit Text Changed!");
}

void PickAndPlacePanel::camera_frame_box_highlighted(int index)
{
    ROS_INFO("Camera Frame Box Highlighted!");
}

void PickAndPlacePanel::camera_frame_box_highlighted(const QString& text)
{
    ROS_INFO("Camera Frame Box Highlighted!");
}

void PickAndPlacePanel::root_frame_box_activated(int index)
{
    // tf_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(point_cloud_sub_, listener_, root_frame_, 2));
    // tf_filter_.registerCallback(std::bind(&PickAndPlaceNode::point_cloud_callback, this, std::placeholders::_1));
}

void PickAndPlacePanel::point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    last_cloud_msg_ = msg;
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
                                         !root_frame_selection_->currentText().isEmpty());
    update_grasps_button_->setEnabled(last_match_.row > 0);
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

void PickAndPlacePanel::gripper_command_action_feedback(GoalHandle goalHandle, const GripperCommandFeedback::ConstPtr& msg)
{
    ROS_INFO("Received feedback");
}

void PickAndPlacePanel::gripper_command_action_transition(GoalHandle goalHandle)
{
    ROS_INFO("Received transition!");
}

} // namespace hdt

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hdt::PickAndPlacePanel, rviz::Panel);