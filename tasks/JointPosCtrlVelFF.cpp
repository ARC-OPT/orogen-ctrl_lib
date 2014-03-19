/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPosCtrlVelFF.hpp"
#include <base/logging.h>

using namespace ctrl_lib;

JointPosCtrlVelFF::JointPosCtrlVelFF(std::string const& name)
    : JointPosCtrlVelFFBase(name)
{
    p_ctrl_ = 0;
}

JointPosCtrlVelFF::JointPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPosCtrlVelFFBase(name, engine)
{
    p_ctrl_ = 0;
}

bool JointPosCtrlVelFF::configureHook()
{
    if (! JointPosCtrlVelFFBase::configureHook())
        return false;

    joint_names_ = _joint_names.get();
    kp_ = _kp.get();
    base::VectorXd max_ctrl_out = _max_ctrl_out.get();

    if(joint_names_.size() != (int)kp_.size())
    {
        LOG_ERROR("Kp property should have size %i but has size %i", joint_names_.size(), kp_.size());
        return false;
    }
    p_ctrl_ = new PDCtrlFeedForward(joint_names_.size());
    p_ctrl_->kp_ = kp_;

    if(max_ctrl_out.size() > 0){
        if(max_ctrl_out.size() != p_ctrl_->no_vars_){
            LOG_ERROR("Max Ctrl out property should have size %i but has size %i", p_ctrl_->no_vars_, max_ctrl_out.size());
            return  false;
        }
        p_ctrl_->v_max_ = max_ctrl_out;
    }

    ctrl_output_.resize(p_ctrl_->no_vars_);
    ctrl_output_.names = joint_names_;
    ctrl_error_.resize(p_ctrl_->no_vars_);
    ctrl_error_.names = joint_names_;

    return true;
}

bool JointPosCtrlVelFF::startHook()
{
    if (! JointPosCtrlVelFFBase::startHook())
        return false;

    return true;
}

void JointPosCtrlVelFF::updateHook()
{
    JointPosCtrlVelFFBase::updateHook();

    if(_setpoint.read(ref_) == RTT::NoData)
    {
        if((base::Time::now() - stamp_).toSeconds() > 1.0)
        {
            LOG_DEBUG("No data on set point port");
            stamp_ = base::Time::now();
        }
        return;
    }

    if(_feedback.read(cur_) == RTT::NoData)
    {
        if((base::Time::now() - stamp_).toSeconds() > 1.0)
        {
            LOG_DEBUG("No data on feedback point port");
            stamp_ = base::Time::now();
        }
        return;
    }

    for(uint i = 0; i < joint_names_.size(); i++)
    {
        uint idx;
        try{
            idx = cur_.mapNameToIndex(joint_names_[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Name %s is not in feedback vector", joint_names_[i].c_str());
            throw std::invalid_argument("Invalid feedback input");
        }

        if(!cur_[idx].hasPosition())
        {
            LOG_ERROR("Feedback vector index %s has invalid position", idx);
            throw std::invalid_argument("Invalid feedback input");
        }

        p_ctrl_->x_(i) = cur_[idx].position;

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

        p_ctrl_->x_r_(i) = ref_[idx].position;
        if(ref_[idx].hasSpeed())
            p_ctrl_->v_r_(i) = ref_[idx].speed;
    }

    //Read new pid values
    if(_kp_values.read(kp_) == RTT::NewData){
        if(!kp_.size() != (int)joint_names_.size())
        {
            LOG_ERROR("Kp Values should have size %i but have size %i", joint_names_.size(), kp_.size());
            throw std::invalid_argument("Invalid gain input");
        }
        p_ctrl_->kp_ = kp_;
    }

    p_ctrl_->updateCtrlOutput();

    for(uint i = 0; i < ctrl_output_.size(); i++){
        ctrl_output_[i].speed = p_ctrl_->v_ctrl_out_(i);
        ctrl_error_[i].speed = p_ctrl_->x_r_(i) - p_ctrl_->x_(i);
    }

    ctrl_output_.time = base::Time::now();
    ctrl_error_.time = base::Time::now();
    _ctrl_out.write(ctrl_output_);
    _control_error.write(ctrl_error_);
}

void JointPosCtrlVelFF::cleanupHook()
{
    JointPosCtrlVelFFBase::cleanupHook();

    delete p_ctrl_;
    p_ctrl_ = 0;
}
