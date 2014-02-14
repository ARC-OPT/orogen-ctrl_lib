/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTPDCTRL_TASK_HPP
#define CTRL_LIB_CARTPDCTRL_TASK_HPP

#include "ctrl_lib/CartPosCtrlVelFFBase.hpp"
#include <ctrl_lib/PDCtrlFeedForward.hpp>

namespace ctrl_lib {
class CartPosCtrlVelFF : public CartPosCtrlVelFFBase
{
    friend class CartPosCtrlVelFFBase;
protected:
    PDCtrlFeedForward *p_ctrl_;
    base::Vector6d kp_;
    base::samples::RigidBodyState ctrl_output_;
    base::samples::RigidBodyState cur_, ref_, ctrl_error_;

    base::Time stamp_;

public:
    CartPosCtrlVelFF(std::string const& name = "ctrl_lib::CartPosCtrlVelFF");
    CartPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartPosCtrlVelFF(){}
    bool configureHook();
    bool startHook(){return CartPosCtrlVelFFBase::startHook();}
    void updateHook();
    void errorHook(){CartPosCtrlVelFFBase::errorHook();}
    void stopHook(){CartPosCtrlVelFFBase::stopHook();}
    void cleanupHook();
};
}

#endif

