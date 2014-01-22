/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPDCtrlFeedForward.hpp"

using namespace ctrl_lib;

CartPDCtrlFeedForward::CartPDCtrlFeedForward(std::string const& name)
    : CartPDCtrlFeedForwardBase(name)
{
    pd_ctrl_ = 0;
}

CartPDCtrlFeedForward::CartPDCtrlFeedForward(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPDCtrlFeedForwardBase(name, engine)
{
    pd_ctrl_ = 0;
}

void CartPDCtrlFeedForward::setPID(const std::vector<base::actuators::PIDValues> &pid){
    if(pid.size() != 6){
        LOG_ERROR("Size of pid parameter is %i but should be 6!", pid.size());
        throw std::invalid_argument("Invalid pid size");
    }

    if(!pd_ctrl_)
        return;

    //Ignores I-Value
    for(uint i = 0; i < 6; i++){
        pd_ctrl_->kp_(i) = pid[i].kp;
        pd_ctrl_->kd_(i) = pid[i].kd;
    }
}

bool CartPDCtrlFeedForward::configureHook()
{
    if (! CartPDCtrlFeedForwardBase::configureHook())
        return false;

    pd_ctrl_ = new PDCtrlFeedForward(6);

    pid_ = _pid.get();
    setPID(pid_);

    base::samples::RigidBodyState max_ctrl_out = _max_ctrl_out.get();

    if(max_ctrl_out.hasValidVelocity())
        pd_ctrl_->v_max_.segment(0,3) = max_ctrl_out.velocity;
    if(max_ctrl_out.hasValidAngularVelocity())
        pd_ctrl_->v_max_.segment(3,3) = max_ctrl_out.angular_velocity;
    if(max_ctrl_out.hasValidAcceleration())
        pd_ctrl_->a_max_.segment(0,3) = max_ctrl_out.acceleration;
    if(max_ctrl_out.hasValidAngularAccelerarion())
        pd_ctrl_->a_max_.segment(3,3) = max_ctrl_out.angular_acceleration;

    return true;
}

void CartPDCtrlFeedForward::updateHook()
{
    CartPDCtrlFeedForwardBase::updateHook();

    //Get Transforms:
    _controlled_in2setpoint.get(base::Time::now(), ref_);
    _controlled_in2controlled_frame.get(base::Time::now(), cur_);

    if(!ref_.hasValidPosition() ||
       !ref_.hasValidOrientation()){
        LOG_DEBUG("Transform between controlled_in and setpoint has no valid position and/or orientation");
        return;
    }

    if(!cur_.hasValidPosition() ||
       !cur_.hasValidOrientation()){
        LOG_DEBUG("Transform between controlled_in and controlled_frame has no valid position and/or orientation");
        return;
    }

    //Convert to eigen:
    pd_ctrl_->x_r_.segment(0,3) = ref_.position;
    pd_ctrl_->x_r_.segment(3,3) = base::getEuler(ref_.orientation);
    pd_ctrl_->x_.segment(0,3) = cur_.position;
    pd_ctrl_->x_.segment(3,3) = base::getEuler(cur_.orientation);

    if(ref_.hasValidVelocity())
        pd_ctrl_->v_r_.segment(0,3) = ref_.velocity;
    if(ref_.hasValidAngularVelocity())
        pd_ctrl_->v_r_.segment(3,3) = ref_.angular_velocity;
    if(cur_.hasValidVelocity())
        pd_ctrl_->v_.segment(0,3) = cur_.velocity;
    if(cur_.hasValidAngularVelocity())
        pd_ctrl_->v_.segment(3,3) = cur_.angular_velocity;
    if(ref_.hasValidAcceleration())
        pd_ctrl_->a_r_.segment(0,3) = ref_.acceleration;
    if(ref_.hasValidAngularAccelerarion())
        pd_ctrl_->a_r_.segment(3,3) = ref_.angular_acceleration;

    //Read new pid values
    if(_pid_values.read(pid_) == RTT::NewData)
        setPID(pid_);

    pd_ctrl_->updateCtrlOutput();

    //Convert to RigidBodyState
    ctrl_output_.velocity = pd_ctrl_->v_ctrl_out_.segment(0,3);
    ctrl_output_.angular_velocity = pd_ctrl_->v_ctrl_out_.segment(3,3);
    ctrl_output_.acceleration = pd_ctrl_->a_ctrl_out_.segment(0,3);
    ctrl_output_.angular_acceleration = pd_ctrl_->a_ctrl_out_.segment(3,3);

    _ctrl_out.write(ctrl_output_);
}

void CartPDCtrlFeedForward::cleanupHook(){
    CartPDCtrlFeedForwardBase::cleanupHook();

    delete pd_ctrl_;
    pd_ctrl_ = 0;
}
