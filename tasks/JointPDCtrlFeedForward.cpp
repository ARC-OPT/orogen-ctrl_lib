/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPDCtrlFeedForward.hpp"
#include <base/logging.h>

using namespace ctrl_lib;

JointPDCtrlFeedForward::JointPDCtrlFeedForward(std::string const& name)
    : JointPDCtrlFeedForwardBase(name)
{
    pd_ctrl_ = 0;
}

JointPDCtrlFeedForward::JointPDCtrlFeedForward(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPDCtrlFeedForwardBase(name, engine)
{
    pd_ctrl_ = 0;
}

void JointPDCtrlFeedForward::setPID(const std::vector<base::actuators::PIDValues> &pid){
    if(pid.size() != 6){
        LOG_ERROR("Size of pid parameter is %i but should be 6!", pid.size());
        throw std::invalid_argument("Invalid pid size");
    }

    if(!pd_ctrl_)
        return;

    //I-Value is ignored
    for(uint i = 0; i < 6; i++){
        pd_ctrl_->kp_(i) = pid[i].kp;
        pd_ctrl_->kd_(i) = pid[i].kd;
    }
}

bool JointPDCtrlFeedForward::configureHook()
{
    if (! JointPDCtrlFeedForwardBase::configureHook())
        return false;

    pid_ = _pid.get();
    pd_ctrl_ = new PDCtrlFeedForward(pid_.size());
    setPID(pid_);

    base::samples::Joints max_ctrl_out = _max_ctrl_out.get();

    if(!max_ctrl_out.empty()){
        if(max_ctrl_out.size() != pd_ctrl_->no_vars_){
            LOG_ERROR("Max Ctrl out property should have size %i but has size %i", pd_ctrl_->no_vars_, max_ctrl_out.size());
            return  false;
        }
        for(uint i = 0; i < pd_ctrl_->no_vars_; i++){
            pd_ctrl_->v_max_(i) = max_ctrl_out[i].speed;
            pd_ctrl_->a_max_(i) = max_ctrl_out[i].effort;
        }
    }

    ctrl_output_.resize(pd_ctrl_->no_vars_);

    return true;
}

bool JointPDCtrlFeedForward::startHook()
{
    if (! JointPDCtrlFeedForwardBase::startHook())
        return false;

    return true;
}

void JointPDCtrlFeedForward::updateHook()
{
    JointPDCtrlFeedForwardBase::updateHook();

    if(_setpoint.read(ref_) == RTT::NoData){
        LOG_DEBUG("No data on set point port");
        return;
    }
    if(ref_.size() != pd_ctrl_->no_vars_){
        LOG_ERROR("Setpoint should have size %i but has size %i", pd_ctrl_->no_vars_, ref_.size());
        throw std::invalid_argument("Invalid Setpoint size");
    }
    if(_feedback.read(cur_) == RTT::NoData){
        LOG_DEBUG("No data on feedback point port");
        return;
    }
    if(cur_.size() != pd_ctrl_->no_vars_){
        LOG_ERROR("Feedback should have size %i but has size %i", pd_ctrl_->no_vars_, cur_.size());
        throw std::invalid_argument("Invalid Feedback size");
    }

    for(uint i = 0; i < pd_ctrl_->no_vars_; i++){
        if(!cur_[i].hasPosition())
            throw std::invalid_argument("Feedback Input has invalid position");

        if(!ref_[i].hasPosition())
            throw std::invalid_argument("Setpoint Input has invalid position");

        pd_ctrl_->x_(i) = cur_[i].position;
        pd_ctrl_->x_r_(i) = ref_[i].position;

        if(cur_[i].hasSpeed())
            pd_ctrl_->v_(i) = cur_[i].speed;
        if(ref_[i].hasSpeed())
            pd_ctrl_->v_r_(i) = ref_[i].speed;

        if(cur_[i].hasEffort())
            pd_ctrl_->a_(i) = cur_[i].effort;
        if(ref_[i].hasEffort())
            pd_ctrl_->a_r_(i) = ref_[i].effort;
    }

    //Read new pid values
    if(_pid_values.read(pid_) == RTT::NewData)
        setPID(pid_);

    pd_ctrl_->updateCtrlOutput();

    if(ctrl_output_.names.empty())
        ctrl_output_.names = ref_.names;

    for(uint i = 0; i < ctrl_output_.size(); i++){
        ctrl_output_[i].speed = pd_ctrl_->v_ctrl_out_(i);
        ctrl_output_[i].effort = pd_ctrl_->a_ctrl_out_(i);
    }

    _ctrl_out.write(ctrl_output_);
}

void JointPDCtrlFeedForward::cleanupHook()
{
    JointPDCtrlFeedForwardBase::cleanupHook();

    delete pd_ctrl_;
    pd_ctrl_ = 0;
}
