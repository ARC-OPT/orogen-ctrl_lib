/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPosCtrlVelFF.hpp"
#include <base/Eigen.hpp>
#include <kdl_conversions/KDLConversions.hpp>

using namespace ctrl_lib;
using namespace std;

CartPosCtrlVelFF::CartPosCtrlVelFF(std::string const& name)
    : CartPosCtrlVelFFBase(name)
{
}

CartPosCtrlVelFF::CartPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPosCtrlVelFFBase(name, engine)
{
}


bool CartPosCtrlVelFF::configureHook()
{
    if (! CartPosCtrlVelFFBase::configureHook())
        return false;

    kp_ = _kp.get();
    kd_ = _kd.get();
    max_ctrl_out_ = _max_ctrl_out.get();
    dead_zone_ = _dead_zone.get();

    return true;
}

bool CartPosCtrlVelFF::startHook(){

    if(!CartPosCtrlVelFFBase::startHook())
        return false;

    ctrl_out_.setZero();
    v_r_.setZero();
    x_r_.setZero();
    x_.setZero();

    return true;
}

void CartPosCtrlVelFF::updateHook()
{
    CartPosCtrlVelFFBase::updateHook();

    //Get Transforms:
    if(!_setpoint2controlled_in.get(base::Time::now(), ref_)){

        if(state() != NO_SETPOINT_TRANSFORM)
            state(NO_SETPOINT_TRANSFORM);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No valid transformation available between %s and %s: No setpoint available.",
                      this->getName().c_str(), _controlled_in_frame.get().c_str(), _setpoint_frame.get().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    if(!_controlled_frame2controlled_in.get(base::Time::now(), cur_)){

        if(state() != NO_END_EFFECTOR_TRANSFORM)
            state(NO_END_EFFECTOR_TRANSFORM);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No valid transformation available between %s and %s.",
                      this->getName().c_str(), _controlled_in_frame.get().c_str(), _controlled_frame_frame.get().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }

    state(RUNNING);

    //Read new controller gain values
    _kp_values.read(kp_);
    _kd_values.read(kd_);

    ref_kdl_.Identity();
    cur_kdl_.Identity();

    //Check valid pose and reference pose
    if(ref_.hasValidPosition() &&
       ref_.hasValidOrientation())
    {
        kdl_conversions::RigidBodyState2KDL(ref_, ref_kdl_);
    }

    if(cur_.hasValidPosition() &&
       cur_.hasValidOrientation()){

        kdl_conversions::RigidBodyState2KDL(cur_, cur_kdl_);
    }

    v_r_.setZero();

    //Check valid reference twist
    if(ref_.hasValidVelocity() &&
       ref_.hasValidAngularVelocity()){
        v_r_.segment(0,3) = ref_.velocity;
        v_r_.segment(3,3) = ref_.angular_velocity;
    }

    //Use KDL::Diff to get a non-singular orientation error
    diff_ = KDL::diff(cur_kdl_, ref_kdl_);

    //Convert to eigen: Use diff as reference value and set current value to zero
    x_r_.segment(0,3) << diff_.vel(0), diff_.vel(1), diff_.vel(2);
    x_r_.segment(3,3) << diff_.rot(0), diff_.rot(1), diff_.rot(2);
    x_.setZero();

    //Compute control error
    ctrl_err_ = x_r_ - x_;

    //Apply dead zone
    for(int i = 0; i < 6; i++){
        if(fabs(ctrl_err_(i)) < dead_zone_(i))
            ctrl_err_(i) = 0;
    }

    //Control law (vel): kd * v_r + kp*(x_r - x)
    ctrl_out_ = kd_.cwiseProduct(v_r_) + kp_.cwiseProduct(ctrl_err_);

    //Apply saturation: ctrl_out = ctrl_out * min(1, max/|ctrl_out|). Scale all entries of ctrl_out appriopriately.
    double eta = 1;
    for(int i = 0; i < 6; i++){
        if(ctrl_out_(i) != 0)
            eta = std::min( eta, max_ctrl_out_(i)/fabs(ctrl_out_(i)) );
    }
    ctrl_out_ = eta * ctrl_out_;

    //Convert to RigidBodyState
    ctrl_out_rbs_.time = base::Time::now();
    ctrl_out_rbs_.velocity = ctrl_out_.segment(0,3);
    ctrl_out_rbs_.angular_velocity = ctrl_out_.segment(3,3);
    ctrl_out_rbs_.sourceFrame = this->getName() + "_ctrl_out_" + cur_.sourceFrame;
    ctrl_out_rbs_.targetFrame = this->getName() + "_ctrl_out_" + ref_.sourceFrame;
    _ctrl_out.write(ctrl_out_rbs_);

}

void CartPosCtrlVelFF::cleanupHook(){
    CartPosCtrlVelFFBase::cleanupHook();
}
