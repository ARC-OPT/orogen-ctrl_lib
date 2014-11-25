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

    base::Vector2d kp_, max_ctrl_out_, dead_zone_;
    camera_axis camera_axis_;
    base::samples::RigidBodyState object2camera_;
    base::commands::Joints ctrl_out_;
    base::Time stamp_;
    double detection_timeout_;

    std::string object_frame_, camera_frame_;

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

