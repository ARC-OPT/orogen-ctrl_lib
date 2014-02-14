/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JointPosCtrlVelFF_TASK_HPP
#define CTRL_LIB_JointPosCtrlVelFF_TASK_HPP

#include "ctrl_lib/JointPosCtrlVelFFBase.hpp"
#include <ctrl_lib/PDCtrlFeedForward.hpp>

namespace ctrl_lib {

class JointPosCtrlVelFF : public JointPosCtrlVelFFBase
{
    friend class JointPosCtrlVelFFBase;
protected:
    PDCtrlFeedForward* p_ctrl_;
    base::VectorXd kp_;
    base::samples::Joints cur_, ref_, ctrl_output_, ctrl_error_;
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

