/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTPIDCTRL_TASK_HPP
#define CTRL_LIB_JOINTPIDCTRL_TASK_HPP

#include "ctrl_lib/JointPIDCtrlBase.hpp"
#include <ctrl_lib/PIDCtrl.hpp>

namespace ctrl_lib {

class JointPIDCtrl : public JointPIDCtrlBase
{
    friend class JointPIDCtrlBase;
protected:
    PIDCtrl* pid_ctrl_;
    std::vector<base::actuators::PIDValues> pid_;
    base::samples::Joints cur_, ref_, ctrl_output_;

    void setPID(const std::vector<base::actuators::PIDValues> &pid);

public:
    JointPIDCtrl(std::string const& name = "ctrl_lib::JointPIDCtrl");
    JointPIDCtrl(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPIDCtrl(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook(){JointPIDCtrlBase::errorHook();}
    void stopHook(){JointPIDCtrlBase::stopHook();}
    void cleanupHook();
};
}

#endif

