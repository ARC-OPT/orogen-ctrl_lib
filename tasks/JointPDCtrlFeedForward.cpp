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
    if(!pd_ctrl_)
        return;

    //I-Value is ignored
    for(uint i = 0; i < pid.size(); i++){
        pd_ctrl_->kp_(i) = pid[i].kp;
        pd_ctrl_->kd_(i) = pid[i].kd;
    }
}

bool JointPDCtrlFeedForward::configureHook()
{
    if (! JointPDCtrlFeedForwardBase::configureHook())
        return false;

    pid_ = _pid.get();
    joint_names_ = _joint_names.get();
    if(joint_names_.size() != pid_.size())
    {
        LOG_ERROR("PID property should have size %i but has size %i", joint_names_.size(), pid_.size());
        return false;
    }
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
    ctrl_error_.resize(pd_ctrl_->no_vars_);

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

    if(_feedback.read(cur_) == RTT::NoData){
        LOG_DEBUG("No data on feedback point port");
        return;
    }

    for(uint i = 0; i < joint_names_.size(); i++)
    {
        uint idx;
        try{
            idx = cur_.mapNameToIndex(joint_names_[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Name %s if not in feedback vector", joint_names_[i].c_str());
            throw std::invalid_argument("Invalid feedback input");
        }

        if(!cur_[idx].hasPosition())
        {
            LOG_ERROR("Feedback vector index %s has invalid position", idx);
            throw std::invalid_argument("Invalid feedback input");
        }

        pd_ctrl_->x_(i) = cur_[idx].position;
        if(cur_[idx].hasSpeed())
            pd_ctrl_->v_(i) = cur_[idx].speed;
        if(cur_[idx].hasEffort())
            pd_ctrl_->a_(i) = cur_[idx].effort;

        try{
            idx = ref_.mapNameToIndex(joint_names_[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Name %s if not in reference vector", joint_names_[i].c_str());
            throw std::invalid_argument("Invalid reference input");
        }

        if(!ref_[idx].hasPosition())
        {
            LOG_ERROR("Reference vector index %s has invalid position", idx);
            throw std::invalid_argument("Invalid reference input");
        }

        pd_ctrl_->x_r_(i) = ref_[idx].position;
        if(ref_[idx].hasSpeed())
            pd_ctrl_->v_r_(i) = ref_[idx].speed;
        if(ref_[idx].hasEffort())
            pd_ctrl_->a_r_(i) = ref_[idx].effort;
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
        ctrl_error_[i].speed = pd_ctrl_->x_r_(i) - pd_ctrl_->x_(i);
        ctrl_error_[i].effort = pd_ctrl_->v_r_(i) - pd_ctrl_->v_(i);
    }

    _ctrl_out.write(ctrl_output_);
    _control_error.write(ctrl_error_);
}

void JointPDCtrlFeedForward::cleanupHook()
{
    JointPDCtrlFeedForwardBase::cleanupHook();

    delete pd_ctrl_;
    pd_ctrl_ = 0;
}
