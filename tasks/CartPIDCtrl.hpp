/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTPIDCTRL_TASK_HPP
#define CTRL_LIB_CARTPIDCTRL_TASK_HPP

#include "ctrl_lib/CartPIDCtrlBase.hpp"
#include <ctrl_lib/PIDCtrl.hpp>

namespace ctrl_lib {
class CartPIDCtrl : public CartPIDCtrlBase
{
    friend class CartPIDCtrlBase;
protected:
    PIDCtrl *pid_ctrl_;
    std::vector<base::actuators::PIDValues> pid_;
    base::samples::RigidBodyState ctrl_output_;
    base::samples::RigidBodyState cur_, ref_;

    void setPID(const std::vector<base::actuators::PIDValues> &pid);

public:
    CartPIDCtrl(std::string const& name = "ctrl_lib::CartPIDCtrl");
    CartPIDCtrl(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartPIDCtrl(){}
    bool configureHook();
    bool startHook(){return CartPIDCtrlBase::startHook();}
    void updateHook();
    void errorHook(){CartPIDCtrlBase::errorHook();}
    void stopHook(){CartPIDCtrlBase::stopHook();}
    void cleanupHook();
};
}

#endif

