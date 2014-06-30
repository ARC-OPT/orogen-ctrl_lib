/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JointPosCtrlVelFF_TASK_HPP
#define CTRL_LIB_JointPosCtrlVelFF_TASK_HPP

#include "ctrl_lib/JointPosCtrlVelFFBase.hpp"

namespace ctrl_lib {

class JointPosCtrlVelFF : public JointPosCtrlVelFFBase
{
    friend class JointPosCtrlVelFFBase;
protected:
    base::VectorXd kp_, kd_;
    base::VectorXd x_, x_r_, v_r_, max_ctrl_out_, ctrl_out_;
    base::samples::Joints cur_, ref_, ctrl_output_;
    std::vector<std::string> joint_names_;
    base::Time stamp_;

public:
    JointPosCtrlVelFF(std::string const& name = "ctrl_lib::JointPosCtrlVelFF");
    JointPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPosCtrlVelFF(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){JointPosCtrlVelFFBase::errorHook();}
    void stopHook(){JointPosCtrlVelFFBase::stopHook();}
    void cleanupHook();
};
}

#endif

