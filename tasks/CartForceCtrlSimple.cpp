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

    if(max_ctrl_output_.size() == 0){
        max_ctrl_output_.resize(6);
        max_ctrl_output_.setConstant(base::infinity<double>());
    }

    if(max_ctrl_output_.size() != 6){
        LOG_ERROR("%s: Size of max_ctrl_out should be 6, but is %i", this->getName().c_str(), max_ctrl_output_.size());
        return false;
    }

    return true;
}

bool CartForceCtrlSimple::startHook()
{
    if (! CartForceCtrlSimpleBase::startHook())
        return false;

    ctrl_error_.setZero();
    ctrl_out_eigen_.setZero();
    ft2refFrame_kdl_ = KDL::Frame::Identity();

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

    if(_wrench_ref.read(wrench_ref_) == RTT::NoData){
        wrench_ref_.force.setZero();
        wrench_ref_.torque.setZero();
    }

    //Convert to KDL
    kdl_conversions::WrenchToKDLWrench(wrench_ref_, F_r_);
    kdl_conversions::WrenchToKDLWrench(wrench_, F_);

    //If there is no transform from ft sensor to reference pt, assume Identity
    if(_ft_sensor2reference.get(base::Time::now(), ft2refFrame_))
        kdl_conversions::RigidBodyState2KDL(ft2refFrame_, ft2refFrame_kdl_);

    cout<<"Pos: "<<ft2refFrame_.position<<endl;
    cout<<"Ori"<<base::getEuler(ft2refFrame_.orientation)<<endl;

    //Ctrl law: kp * (F_r - F)
    if(_ctrl_output_in_ref_frame)
        KDLWrench2Eigen(F_r_ -  ft2refFrame_kdl_.Inverse() * F_, ctrl_error_);
    else
        KDLWrench2Eigen(ft2refFrame_kdl_ * F_r_ -  F_, ctrl_error_);

    cout<<"Wrench in ft frame: "<<ft2refFrame_kdl_ * F_r_<<endl;
    cout<<"Ft Frame to Hand frame: "<<ft2refFrame_kdl_<<endl<<endl;

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

    ctrl_out_.time = base::Time::now();
    _ctrl_out.write(ctrl_out_);
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
