/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTPDCTRL_TASK_HPP
#define CTRL_LIB_CARTPDCTRL_TASK_HPP

#include "ctrl_lib/CartPosCtrlVelFFBase.hpp"
#include <kdl/frames_io.hpp>

namespace ctrl_lib {
class CartPosCtrlVelFF : public CartPosCtrlVelFFBase
{
    friend class CartPosCtrlVelFFBase;
protected:
    base::Vector6d kp_, kd_;
    base::Vector6d max_ctrl_out_;
    base::Vector6d dead_zone_;
    base::Vector6d ctrl_out_;
    base::Vector6d x_r_, x_, v_r_, ctrl_err_;
    KDL::Frame ref_kdl_, cur_kdl_;
    base::samples::RigidBodyState ctrl_out_rbs_;
    base::samples::RigidBodyState cur_, ref_, pos_ctrl_error_;
    KDL::Twist diff_;

    base::Time stamp_;

public:
    CartPosCtrlVelFF(std::string const& name = "ctrl_lib::CartPosCtrlVelFF");
    CartPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartPosCtrlVelFF(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void control_step_and_write();
    void errorHook(){CartPosCtrlVelFFBase::errorHook();}
    void stopHook(){CartPosCtrlVelFFBase::stopHook();}
    void cleanupHook();
};
}

#endif

