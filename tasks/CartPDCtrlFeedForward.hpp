/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTPDCTRL_TASK_HPP
#define CTRL_LIB_CARTPDCTRL_TASK_HPP

#include "ctrl_lib/CartPDCtrlFeedForwardBase.hpp"
#include <ctrl_lib/PDCtrlFeedForward.hpp>

namespace ctrl_lib {
class CartPDCtrlFeedForward : public CartPDCtrlFeedForwardBase
{
    friend class CartPDCtrlFeedForwardBase;
protected:
    PDCtrlFeedForward *pd_ctrl_;
    std::vector<base::actuators::PIDValues> pid_;
    base::samples::RigidBodyState ctrl_output_;
    base::samples::RigidBodyState cur_, ref_, ctrl_error_;

    void setPID(const std::vector<base::actuators::PIDValues> &pid);

public:
    CartPDCtrlFeedForward(std::string const& name = "ctrl_lib::CartPDCtrlFeedForward");
    CartPDCtrlFeedForward(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartPDCtrlFeedForward(){}
    bool configureHook();
    bool startHook(){return CartPDCtrlFeedForwardBase::startHook();}
    void updateHook();
    void errorHook(){CartPDCtrlFeedForwardBase::errorHook();}
    void stopHook(){CartPDCtrlFeedForwardBase::stopHook();}
    void cleanupHook();
};
}

#endif

