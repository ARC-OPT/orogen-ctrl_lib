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
    camera_axis_ = z_axis;// _camera_axis.get();

    return true;
}
bool CartGazeCtrl::startHook()
{
    if (! CartGazeCtrlBase::startHook())
        return false;

    ctrl_out_rbs_.velocity.setZero();
    ctrl_out_rbs_.angular_velocity.setZero();

    return true;
}
void CartGazeCtrl::updateHook()
{
    CartGazeCtrlBase::updateHook();

    //Get Transforms:
    if(!_object2camera.get(base::Time::now(), ref_)){
        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No valid transformation available between %s and %s: No setpoint available.",
                      this->getName().c_str(), _controlled_in_frame.get().c_str(), _setpoint_frame.get().c_str());
            stamp_ = base::Time::now();
        }
        if(state() != WAITING_FOR_TRANSFORM_OBJECT2CAMERA)
            state(WAITING_FOR_TRANSFORM_OBJECT2CAMERA);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    //Make sure transforms are valid:
    if(!ref_.hasValidPosition())
    {
        LOG_ERROR("%s: Reference pose (sourceFrame: %s, TargetFrame: %s) has invalid position.",
                  this->getName().c_str(), ref_.sourceFrame.c_str(), ref_.targetFrame.c_str());
        throw std::invalid_argument("Invalid transform");
    }

    //Convert position error to orientation error
    switch(camera_axis_)
    {
    case x_axis:{
        LOG_ERROR("Only z-axis as camera axis is supported right now");
        throw std::invalid_argument("TODO: Implement me!");
        break;
    }
    case y_axis:{
        LOG_ERROR("Only z-axis as camera axis is supported right now");
        throw std::invalid_argument("TODO: Implement me!");
        break;
    }
    case z_axis:{
        ctrl_out_rbs_.angular_velocity(0) = kp_(0) * atan2(-ref_.position(1), ref_.position(2)); //x-axis angular error: phi_x = atan2(-dy, dz)
        ctrl_out_rbs_.angular_velocity(1) = kp_(1) * atan2(ref_.position(0), ref_.position(2)); //y-axis angular error: phi_y = atan2(dx, dz)
        ctrl_out_rbs_.angular_velocity(2) = 0;
        break;
    }

    }

    //Apply saturation: ctrl_out = ctrl_out * min(1, max/|ctrl_out|). Scale all entries of ctrl_out appriopriately.
    double eta = 1;
    for(int i = 0; i < 3; i++){
        if(ctrl_out_rbs_.angular_velocity(i) != 0)
            eta = std::min( eta, max_ctrl_out_(i)/fabs(ctrl_out_rbs_.angular_velocity(i)) );
    }

    for(int i = 0; i < 3; i++)
        ctrl_out_rbs_.angular_velocity(i) = eta * ctrl_out_rbs_.angular_velocity(i);

    //Convert to RigidBodyState*/
    ctrl_out_rbs_.time = base::Time::now();

    _ctrl_out.write(ctrl_out_rbs_);

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
