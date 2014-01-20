/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPIDCtrl.hpp"

using namespace ctrl_lib;

CartPIDCtrl::CartPIDCtrl(std::string const& name)
    : CartPIDCtrlBase(name)
{
    pid_ctrl_ = 0;
}

CartPIDCtrl::CartPIDCtrl(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPIDCtrlBase(name, engine)
{
    pid_ctrl_ = 0;
}

void CartPIDCtrl::setPID(const std::vector<base::actuators::PIDValues> &pid){
    if(pid.size() != 6){
        LOG_ERROR("Size of pid parameter is %i but should be 6!", pid.size());
        throw std::invalid_argument("Invalid pid size");
    }

    if(!pid_ctrl_)
        return;

    for(uint i = 0; i < 6; i++){
        pid_ctrl_->kp_(i) = pid[i].kp;
        pid_ctrl_->ki_(i) = pid[i].ki;
        pid_ctrl_->kd_(i) = pid[i].kd;
        pid_ctrl_->windup_(i) = pid[i].maxPWM;
    }
}

bool CartPIDCtrl::configureHook()
{
    if (! CartPIDCtrlBase::configureHook())
        return false;

    pid_ctrl_ = new PIDCtrl(6);

    pid_ = _pid.get();
    setPID(pid_);

    base::samples::RigidBodyState max_ctrl_out = _max_ctrl_out.get();

    if(max_ctrl_out.hasValidVelocity())
        pid_ctrl_->v_max_.segment(0,3) = max_ctrl_out.velocity;
    if(max_ctrl_out.hasValidAngularVelocity())
        pid_ctrl_->v_max_.segment(3,3) = max_ctrl_out.angular_velocity;
    if(max_ctrl_out.hasValidAcceleration())
        pid_ctrl_->a_max_.segment(0,3) = max_ctrl_out.acceleration;
    if(max_ctrl_out.hasValidAngularAccelerarion())
        pid_ctrl_->a_max_.segment(3,3) = max_ctrl_out.angular_acceleration;


    pid_ctrl_->pos_err_sum_.setZero();

    return true;
}

void CartPIDCtrl::updateHook()
{
    CartPIDCtrlBase::updateHook();

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
    pid_ctrl_->x_r_.segment(0,3) = ref_.position;
    pid_ctrl_->x_r_.segment(3,3) = base::getEuler(ref_.orientation);
    pid_ctrl_->x_.segment(0,3) = cur_.position;
    pid_ctrl_->x_.segment(3,3) = base::getEuler(cur_.orientation);

    if(ref_.hasValidVelocity())
        pid_ctrl_->v_r_.segment(0,3) = ref_.velocity;
    if(ref_.hasValidAngularVelocity())
        pid_ctrl_->v_r_.segment(3,3) = ref_.angular_velocity;
    if(cur_.hasValidVelocity())
        pid_ctrl_->v_.segment(0,3) = cur_.velocity;
    if(cur_.hasValidAngularVelocity())
        pid_ctrl_->v_.segment(3,3) = cur_.angular_velocity;
    if(ref_.hasValidAcceleration())
        pid_ctrl_->a_r_.segment(0,3) = ref_.acceleration;
    if(ref_.hasValidAngularAccelerarion())
        pid_ctrl_->a_r_.segment(3,3) = ref_.angular_acceleration;

    //Read new pid values
    if(_pid_values.read(pid_) == RTT::NewData)
        setPID(pid_);

    pid_ctrl_->updateCtrlOutput();

    //Convert to RigidBodyState
    ctrl_output_.velocity = pid_ctrl_->v_ctrl_out_.segment(0,3);
    ctrl_output_.angular_velocity = pid_ctrl_->v_ctrl_out_.segment(3,3);
    ctrl_output_.acceleration = pid_ctrl_->a_ctrl_out_.segment(0,3);
    ctrl_output_.angular_acceleration = pid_ctrl_->a_ctrl_out_.segment(3,3);

    _ctrl_out.write(ctrl_output_);
}

void CartPIDCtrl::cleanupHook(){
    CartPIDCtrlBase::cleanupHook();

    delete pid_ctrl_;
    pid_ctrl_ = 0;
}
