add_executable(
    repeatability_measure
    repeatability_measure.cpp
    RepeatabilityMeasurementNode.cpp)
target_link_libraries(
    repeatability_measure
    ${actionlib_LIBRARIES}
    ${eigen_conversions_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${spellbook_ros_LIBRARIES}
    ${tf_LIBRARIES}
    hdt_robot_model)
