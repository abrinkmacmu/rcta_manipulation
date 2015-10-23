#ifndef hdt_HDTRobotModel_h
#define hdt_HDTRobotModel_h

#include <ros/ros.h>
#include <Eigen/Dense>
#include <hdt_description/RobotModel.h>
#include <sbpl_manipulation_components/robot_model.h>
#include <urdf/model.h>
#include <eigen_conversions/eigen_msg.h>

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
    bool computeIK(const std::vector<double> &pose, const std::vector<double> &start, std::vector< std::vector<double> > &solutions, int option=0);

    double computeIKscore(const std::vector<double> &ik) ;

private:

    ros::NodeHandle nh_;
    ros::Publisher pub_;

    hdt::RobotModelPtr robot_model_;

    Eigen::Affine3d mount_frame_to_manipulator_frame_;

    // Privatizing these methods since the OpenRAVE IK is generated only for the 7th dof
    RobotModel::setPlanningJoints;
    RobotModel::setPlanningLink;

    //sorts the vector of ik solutions -- best ones should be at the end!
    struct iksortstruct
    {
      // sortstruct needs to know its containing object
      HDTRobotModel* rm_;
      bool sort_asc;
      iksortstruct(HDTRobotModel* rm, bool asc) : rm_(rm), sort_asc(asc) {};
 
      bool operator() ( const std::vector<double> &ik1, const std::vector<double> &ik2 ) const
      {
        double s1 = rm_->computeIKscore(ik1);
        double s2 = rm_->computeIKscore(ik2);
        if ( sort_asc )
	  return s1 < s2;
        else return s1 > s2;
      }
    };

    void sortIKsolutions(std::vector<std::vector<double> > &solutions);
};

}

#endif

