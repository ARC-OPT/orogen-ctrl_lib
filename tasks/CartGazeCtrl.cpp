/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartGazeCtrl.hpp"
#include <base/logging.h>
#include <kdl/frames_io.hpp>

using namespace ctrl_lib;
using namespace std;

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
    camera_axis_ = _camera_axis.get();
    detection_timeout_ = _detection_timeout.get();
    dead_zone_ = _dead_zone.get();
    object_frame_ = _object_frame.get();
    camera_frame_ = _camera_frame.get();

    return true;
}
bool CartGazeCtrl::startHook()
{
    if (! CartGazeCtrlBase::startHook())
        return false;

    ctrl_out_.velocity.setZero();
    ctrl_out_.angular_velocity.setZero();

    return true;
}
void CartGazeCtrl::updateHook()
{
    CartGazeCtrlBase::updateHook();

    //Get Transform
    bool has_transform = _object2camera.get(base::Time::now(), object2camera_);

    base::Time cur = base::Time::now();

    //if there is no transform available OR the difference between transform timestamp and current time is bigger
    //than detection_timeout, write zero velocities to port and go into NO_OBJECT_TRANSFORM state
    if(!has_transform || (cur - object2camera_.time).toSeconds() > detection_timeout_)
    {
        if((cur-stamp_).toSeconds() > 2) //Only print warn msg every 2 seconds
        {
            LOG_WARN("%s: No valid transformation available between %s and %s: No setpoint available.",
                      this->getName().c_str(), object_frame_.c_str(), camera_frame_.c_str());
        }

        stamp_ = cur;

        ctrl_out_.angular_velocity.setZero();
        ctrl_out_.time = cur;
        _ctrl_out.write(ctrl_out_);

        if(state() != NO_OBJECT_TRANSFORM) //only set NO_OBJECT_TRANSFORM state if we are NOT in NO_OBJECT_TRANSFORM state
            state(NO_OBJECT_TRANSFORM);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    //Make sure transform is valid:
    if(!object2camera_.hasValidPosition())
    {
        LOG_ERROR("%s: Reference pose (sourceFrame: %s, TargetFrame: %s) has invalid position.",
                  this->getName().c_str(), object2camera_.sourceFrame.c_str(), object2camera_.targetFrame.c_str());
        throw std::invalid_argument("Invalid transform");
    }

    switch(camera_axis_)
    {
    case x_axis: throw std::invalid_argument("Not implemented yet"); break;
    case y_axis: throw std::invalid_argument("Not implemented yet"); break;
    case z_axis: {
        ctrl_out_.angular_velocity(0) = kp_(1) * atan2(object2camera_.position(0), object2camera_.position(2)); //y-axis angular error: phi_y = atan2(dx, dz)
        ctrl_out_.angular_velocity(1) = kp_(0) * atan2(-object2camera_.position(1), object2camera_.position(2)); //x-axis angular error: phi_x = atan2(-dy, dz)
        break;
    }
    default: throw std::invalid_argument("Invalid camera axis");
    }

    //Apply dead zone
    for(uint i = 0; i < 3; i++){
        if(fabs(ctrl_out_.angular_velocity(i)) < dead_zone_(i))
            ctrl_out_.angular_velocity(i) = 0;
    }

    //Apply controller saturation: ctrl_out = ctrl_out * min(1, max/|ctrl_out|). Scale all entries of ctrl_out appriopriately.
    double eta = 1;
    for(uint i = 0; i < 3; i++){
        if(ctrl_out_.angular_velocity(i) != 0)
            eta = std::min( eta, max_ctrl_out_(i)/fabs(ctrl_out_.angular_velocity(i)) );
    }
    for(uint i = 0; i < 3; i++)
        ctrl_out_.angular_velocity(i) = eta * ctrl_out_.angular_velocity(i);

    //Write to port
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);

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
