/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTPDCTRLFEEDFORWARD_TASK_HPP
#define CTRL_LIB_JOINTPDCTRLFEEDFORWARD_TASK_HPP

#include "ctrl_lib/JointPDCtrlFeedForwardBase.hpp"
#include <ctrl_lib/PDCtrlFeedForward.hpp>

namespace ctrl_lib {

class JointPDCtrlFeedForward : public JointPDCtrlFeedForwardBase
{
    friend class JointPDCtrlFeedForwardBase;
protected:
    PDCtrlFeedForward* pd_ctrl_;
    std::vector<base::actuators::PIDValues> pid_;
    base::samples::Joints cur_, ref_, ctrl_output_, ctrl_error_;

    void setPID(const std::vector<base::actuators::PIDValues> &pid);

public:
    JointPDCtrlFeedForward(std::string const& name = "ctrl_lib::JointPDCtrlFeedForward");
    JointPDCtrlFeedForward(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPDCtrlFeedForward(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){JointPDCtrlFeedForwardBase::errorHook();}
    void stopHook(){JointPDCtrlFeedForwardBase::stopHook();}
    void cleanupHook();
};
}

#endif

