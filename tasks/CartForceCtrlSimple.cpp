/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartForceCtrlSimple.hpp"
#include <kdl/frames_io.hpp>

using namespace ctrl_lib;
using namespace std;

CartForceCtrlSimple::CartForceCtrlSimple(std::string const& name)
    : CartForceCtrlSimpleBase(name)
{
}

CartForceCtrlSimple::CartForceCtrlSimple(std::string const& name, RTT::ExecutionEngine* engine)
    : CartForceCtrlSimpleBase(name, engine)
{
}

CartForceCtrlSimple::~CartForceCtrlSimple()
{
}

bool CartForceCtrlSimple::configureHook()
{
    if (! CartForceCtrlSimpleBase::configureHook())
        return false;

    kp_ = _kp.get();
    max_ctrl_output_ = _max_ctrl_out.get();
    dead_zone_ = _dead_zone.get();

    activation_.resize(6);

    return true;
}

bool CartForceCtrlSimple::startHook()
{
    if (! CartForceCtrlSimpleBase::startHook())
        return false;

    ctrl_error_.setZero();
    ctrl_out_eigen_.setZero();
    activation_.setZero();

    return true;
}

void CartForceCtrlSimple::updateHook()
{
    CartForceCtrlSimpleBase::updateHook();

    if(_wrench.read(wrench_) == RTT::NoData){

        if(state() != NO_WRENCH_ON_PORT)
            state(NO_WRENCH_ON_PORT);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No Data on wrench port", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    validateWrench(wrench_);

    if(_wrench_ref.read(wrench_ref_) == RTT::NoData){

        if(state() != NO_REF_WRENCH_ON_PORT)
            state(NO_REF_WRENCH_ON_PORT);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No Data on wrench_ref port", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    validateWrench(wrench_ref_);

    _kp_values.read(kp_);

    if(!_wrench2controlled_in.get(base::Time::now(), ft2baseFrame_)){

        if(state() != NO_FT_SENSOR_TRANSFORM)
            state(NO_FT_SENSOR_TRANSFORM);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: Missing transform from force sensor to base frame", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    else
        kdl_conversions::RigidBodyState2KDL(ft2baseFrame_, ft2baseFrame_kdl_);

    if(!_wrench_ref2controlled_in.get(base::Time::now(), ref2baseFrame_)){

        if(state() != NO_SETPOINT_TRANSFORM)
            state(NO_SETPOINT_TRANSFORM);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: Missing transform from force sensor to base frame", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    else
        kdl_conversions::RigidBodyState2KDL(ref2baseFrame_, ref2baseFrame_kdl_);

    state(RUNNING);

    //Convert to KDL
    kdl_conversions::WrenchToKDLWrench(wrench_ref_, F_r_);
    kdl_conversions::WrenchToKDLWrench(wrench_, F_);

    //Ctrl law: kp * (F_r - F)
    KDLWrench2Eigen(ref2baseFrame_kdl_ * F_r_ -  ft2baseFrame_kdl_ * F_, ctrl_error_);

    //Apply dead zone
    for(int i = 0; i < 6; i++)
    {
        if(ctrl_error_(i) > dead_zone_(i))
            ctrl_error_(i) -= dead_zone_(i);
        else if(ctrl_error_(i) < -dead_zone_(i))
            ctrl_error_(i) += dead_zone_(i);
        else{
            ctrl_error_(i) = 0;
        }
    }

    //Multiply gain
    ctrl_out_eigen_ = kp_.cwiseProduct(ctrl_error_);

    //Apply saturation: ctrl_out = ctrl_out * min(1, max/|ctrl_out|). Scale all entries of ctrl_out appriopriately.
    double eta = 1;
    for(int i = 0; i < 6; i++){
        if(ctrl_out_eigen_(i) != 0)
            eta = std::min( eta, max_ctrl_output_(i)/fabs(ctrl_out_eigen_(i)) );
    }
    ctrl_out_eigen_ = eta * ctrl_out_eigen_;

    //Convert back to RigidBodyState
    ctrl_out_.velocity = ctrl_out_eigen_.segment(0,3);
    ctrl_out_.angular_velocity = ctrl_out_eigen_.segment(3,3);

    ctrl_out_.sourceFrame = this->getName() + "_ctrl_out_" + ft2baseFrame_.sourceFrame;
    ctrl_out_.targetFrame = this->getName() + "_ctrl_out_" + ft2baseFrame_.targetFrame;
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);


    //Set activation: Activate all force or torque dof if one direction is non-zero, since usually we will transform the output into the robot's base frame
    //where the output might act on more than one direction
    activation_.setZero();
    for(int i = 0; i < 3; i++){
        if(ctrl_error_(i) != 0){
            activation_.segment(0,3).setConstant(1);
            break;
        }
    }
    for(int i = 3; i < 6; i++){
        if(ctrl_error_(i) != 0){
            activation_.segment(3,3).setConstant(1);
            break;
        }
    }
    _activation.write(activation_);
}

void CartForceCtrlSimple::errorHook()
{
    CartForceCtrlSimpleBase::errorHook();
}

void CartForceCtrlSimple::stopHook()
{
    CartForceCtrlSimpleBase::stopHook();
}

void CartForceCtrlSimple::cleanupHook()
{
    CartForceCtrlSimpleBase::cleanupHook();
}
