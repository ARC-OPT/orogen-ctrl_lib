/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_MANIPULABILITYGRADIENTCTRL_TASK_HPP
#define CTRL_LIB_MANIPULABILITYGRADIENTCTRL_TASK_HPP

#include "ctrl_lib/ManipulabilityGradientCtrlBase.hpp"
#include <kdl/chainjnttojacsolver.hpp>
#include <base/commands/Joints.hpp>

namespace ctrl_lib {

class ManipulabilityGradientCtrl : public ManipulabilityGradientCtrlBase
{
    friend class ManipulabilityGradientCtrlBase;
protected:
    double delta_q_;
    KDL::ChainJntToJacSolver* jac_solver_;
    KDL::JntArray q_;
    base::VectorXd kp_;
    base::commands::Joints ctrl_out_;
    base::samples::Joints joint_state_;
    KDL::Jacobian jac_;
    double manipulability(const KDL::JntArray &q);

public:
    ManipulabilityGradientCtrl(std::string const& name = "ctrl_lib::ManipulabilityGradientCtrl");
    ManipulabilityGradientCtrl(std::string const& name, RTT::ExecutionEngine* engine);
    ~ManipulabilityGradientCtrl();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

