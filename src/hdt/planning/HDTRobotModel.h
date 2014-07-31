#ifndef hdt_HDTRobotModel_h
#define hdt_HDTRobotModel_h

#include <Eigen/Dense>
#include <hdt_description/RobotModel.h>
#include <sbpl_manipulation_components/robot_model.h>
#include <urdf/model.h>

namespace hdt
{

class HDTRobotModel : public sbpl_arm_planner::RobotModel
{
public:

    HDTRobotModel();
    ~HDTRobotModel();

    ///@{ Inherited RobotModel API
    /// virtual bool init(std::string robot_description, std::vector<std::string> &planning_joints);
    /// void setPlanningJoints(const std::vector<std::string> &joints);
    /// void setPlanningLink(std::string name);
    /// std::string getPlanningLink();
    /// void setPlanningFrame(std::string name);
    /// std::string getPlanningFrame();
    /// void getKinematicsFrame(std::string &name);
    /// void setLoggerName(std::string name);
    /// void setKinematicsToPlanningTransform(const KDL::Frame &f, std::string name);
    ///@}

    ///@{ Unimplemented Virtual Functions from Robot Model
    /// virtual bool checkJointLimits(const std::vector<double> &angles);
    /// virtual bool computeFK(const std::vector<double> &angles, std::string name, KDL::Frame &f);
    /// virtual bool computeFK(const std::vector<double> &angles, std::string name, std::vector<double> &pose);
    /// virtual bool computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose);
    /// virtual bool computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, int option=0);
    /// virtual bool computeFastIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution);
    /// virtual void printRobotModelInformation();
    ///@}

    bool init(const std::string& robot_description);

    /* Joint Limits */
    bool checkJointLimits(const std::vector<double>& angles);

    /* Forward Kinematics */
    bool computePlanningLinkFK(const std::vector<double> &angles, std::vector<double> &pose);

    /* Inverse Kinematics */
    bool computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector<double> &solution, int option=0);

private:

    hdt::RobotModel robot_model_;

    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    int free_angle_idx_;

    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    // Privatizing these methods since the OpenRAVE IK is generated only for the 7th dof
    RobotModel::init;
    RobotModel::setPlanningJoints;
    RobotModel::setPlanningLink;
};

}

#endif

