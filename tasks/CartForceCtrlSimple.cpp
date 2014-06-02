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
    base::samples::Wrench contact_thresh = _contact_threshold.get();

    if(contact_thresh.force.size() == 3)
        contact_threshold_.segment(0,3) = contact_thresh.force;
    else
        contact_threshold_.segment(0,3).setZero();

    if(contact_thresh.torque.size() == 3)
        contact_threshold_.segment(3,3) = contact_thresh.torque;
    else
        contact_threshold_.segment(3,3).setZero();

    if(max_ctrl_output_.size() == 0){
        max_ctrl_output_.resize(6);
        max_ctrl_output_.setConstant(base::infinity<double>());
    }

    if(max_ctrl_output_.size() != 6){
        LOG_ERROR("%s: Size of max_ctrl_out should be 6, but is %i", this->getName().c_str(), max_ctrl_output_.size());
        return false;
    }

    activation_.resize(6);

    return true;
}

bool CartForceCtrlSimple::startHook()
{
    if (! CartForceCtrlSimpleBase::startHook())
        return false;

    ctrl_error_.setZero();
    ctrl_out_eigen_.setZero();
    ft2refFrame_kdl_ = KDL::Frame::Identity();
    activation_.setZero();

    return true;
}

void CartForceCtrlSimple::updateHook()
{
    CartForceCtrlSimpleBase::updateHook();

    if(_wrench.read(wrench_) == RTT::NoData){
        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No Data on wrench port", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    validateWrench(wrench_);

    _wrench_out.write(wrench_);

    if(_wrench_ref.read(wrench_ref_) == RTT::NoData){
        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No Data on wrench_ref port", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    validateWrench(wrench_ref_);

    //Convert to KDL
    kdl_conversions::WrenchToKDLWrench(wrench_ref_, F_r_);
    kdl_conversions::WrenchToKDLWrench(wrench_, F_);

    //If there is no transform from ft sensor to reference pt, assume Identity
    if(_ft_sensor2reference.get(base::Time::now(), ft2refFrame_))
        kdl_conversions::RigidBodyState2KDL(ft2refFrame_, ft2refFrame_kdl_);

    //Ctrl law: kp * (F_r - F)
    if(_ctrl_output_in_ref_frame){
        KDL::Wrench tmp = ft2refFrame_kdl_ * F_;
        KDLWrench2Eigen(F_r_ -  tmp, ctrl_error_);
    }
    else{
        KDL::Wrench tmp = ft2refFrame_kdl_.Inverse() * F_r_;
        KDLWrench2Eigen(tmp -  F_, ctrl_error_);
    }

    //Apply contact threshold
    for(int i = 0; i < 6; i++)
    {
        if(ctrl_error_(i) > contact_threshold_(i))
            ctrl_error_(i) -= contact_threshold_(i);
        else if(ctrl_error_(i) < -contact_threshold_(i))
            ctrl_error_(i) += contact_threshold_(i);
        else{
            ctrl_error_(i) = 0;
        }
    }

    //Set activation: Activate all force or torque dof if one direction is non-zero, since usually we will transform the output into the robot's base frame
    //where the output might act on more than one direction
    activation_.setZero();
    for(int i = 0; i < 3; i++)
    {
        if(ctrl_error_(i) != 0){
            activation_.segment(0,3).setConstant(1);
            break;
        }
    }
    for(int i = 3; i < 6; i++)
    {
        if(ctrl_error_(i) != 0){
            activation_.segment(3,3).setConstant(1);
            break;
        }
    }

    _ctrl_error.write(ctrl_error_);

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

    ctrl_out_.sourceFrame = this->getName() + "_ctrl_out_" + ft2refFrame_.sourceFrame;
    ctrl_out_.targetFrame = this->getName() + "_ctrl_out_" + ft2refFrame_.targetFrame;
    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);

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
