/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartGazeCtrl.hpp"
#include <base/logging.h>
#include <kdl/frames_io.hpp>

using namespace ctrl_lib;

CartGazeCtrl::CartGazeCtrl(std::string const& name)
    : CartGazeCtrlBase(name)
{
}

CartGazeCtrl::CartGazeCtrl(std::string const& name, RTT::ExecutionEngine* engine)
    : CartGazeCtrlBase(name, engine)
{
}

CartGazeCtrl::~CartGazeCtrl()
{
}

bool CartGazeCtrl::configureHook()
{
    if (! CartGazeCtrlBase::configureHook())
        return false;

    kp_ = _kp.get();
    max_ctrl_out_ = _max_ctrl_out.get();
    joint_names_ = _joint_names.get();

    ctrl_out_cmd_.resize(joint_names_.size());
    ctrl_out_cmd_.names = joint_names_;

    return true;
}
bool CartGazeCtrl::startHook()
{
    if (! CartGazeCtrlBase::startHook())
        return false;
    ctrl_out_.setZero();
    return true;
}
void CartGazeCtrl::updateHook()
{
    CartGazeCtrlBase::updateHook();

    //Get Transforms:
    if(!_controlled_in2setpoint.get(base::Time::now(), ref_)){
        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No valid transformation available between %s and %s: No setpoint available.",
                      this->getName().c_str(), _controlled_in_frame.get().c_str(), _setpoint_frame.get().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }

    //Make sure transforms are valid:
    if(!ref_.hasValidPosition())
    {
        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: Reference pose (sourceFrame: %s, TargetFrame: %s) has invalid position.",
                      this->getName().c_str(), ref_.sourceFrame.c_str(), ref_.targetFrame.c_str());
            stamp_ = base::Time::now();
            return;
        }
    }

    //Convert position error to orientation error
    ctrl_out_cmd_[0].speed = kp_(0) * atan2(ref_.position(0), ref_.position(2)); //x-axis angular error: phi_x = atan2(-dy, dz)
    ctrl_out_cmd_[1].speed = kp_(1) * atan2(-ref_.position(1), ref_.position(2)); //y-axis angular error: phi_y = atan2(dx, dz)

    //Apply saturation: ctrl_out = ctrl_out * min(1, max/|ctrl_out|). Scale all entries of ctrl_out appriopriately.
    double eta = 1;
    for(int i = 0; i < 2; i++){
        if(ctrl_out_cmd_[i].speed != 0)
            eta = std::min( eta, max_ctrl_out_(i)/fabs(ctrl_out_cmd_[i].speed) );
    }

    for(int i = 0; i < 2; i++)
        ctrl_out_cmd_[i].speed = eta * ctrl_out_cmd_[i].speed;

    //Convert to RigidBodyState*/
    ctrl_out_cmd_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_cmd_);

}
void CartGazeCtrl::errorHook()
{
    CartGazeCtrlBase::errorHook();
}
void CartGazeCtrl::stopHook()
{
    CartGazeCtrlBase::stopHook();
}
void CartGazeCtrl::cleanupHook()
{
    CartGazeCtrlBase::cleanupHook();
}
