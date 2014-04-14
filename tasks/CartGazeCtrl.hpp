/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CARTGAZECTRL_TASK_HPP
#define CTRL_LIB_CARTGAZECTRL_TASK_HPP

#include "ctrl_lib/CartGazeCtrlBase.hpp"
#include <kdl_conversions/KDLConversions.hpp>

namespace ctrl_lib {
class CartGazeCtrl : public CartGazeCtrlBase
{
    friend class CartGazeCtrlBase;
protected:

    std::vector<std::string> joint_names_;
    base::Vector2d kp_, max_ctrl_out_, ctrl_out_;
    base::samples::RigidBodyState ref_;
    base::samples::Joints ctrl_out_cmd_;
    KDL::Twist diff_;
    base::Time stamp_;

public:
    CartGazeCtrl(std::string const& name = "ctrl_lib::CartGazeCtrl");
    CartGazeCtrl(std::string const& name, RTT::ExecutionEngine* engine);
    ~CartGazeCtrl();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

