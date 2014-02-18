/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointRepPotField.hpp"
#include <base/logging.h>

using namespace ctrl_lib;

JointRepPotField::JointRepPotField(std::string const& name)
    : JointRepPotFieldBase(name)
{
}

JointRepPotField::JointRepPotField(std::string const& name, RTT::ExecutionEngine* engine)
    : JointRepPotFieldBase(name, engine)
{
}

bool JointRepPotField::configureHook()
{
    if (! JointRepPotFieldBase::configureHook())
        return false;

    std::vector<std::string> joint_names = _joint_names.get();
    base::VectorXd q_zero = _q_zero.get();
    base::VectorXd d_zero = _d_zero.get();
    base::VectorXd kp = _kp.get();
    base::VectorXd max_ctrl_out = _max_ctrl_out.get();
    transition_range_ = _transition_range.get();

    if(q_zero.size() != (int)joint_names.size()){
        LOG_ERROR("q_zero property should have size %i, but has size %i", joint_names.size(), q_zero.size());
        return false;
    }
    if(d_zero.size() != (int)joint_names.size()){
        LOG_ERROR("d_zero property should have size %i, but has size %i", joint_names.size(), d_zero.size());
        return false;
    }
    if(kp.size() != (int)joint_names.size()){
        LOG_ERROR("d_zero property should have size %i, but has size %i", joint_names.size(), kp.size());
        return false;
    }

    if(transition_range_ < 0 || transition_range_ > 1){
        LOG_ERROR("Transition range must be within 0 .. 1");
        return false;
    }

    for(uint i = 0; i < joint_names.size(); i++){
        RepulsivePotentialField* rpf = new RepulsivePotentialField(1);
        rpf->d0_ = d_zero(i);
        rpf->q0_(0) = q_zero(i);
        rpf->kp_(0) = kp(i);
        if(max_ctrl_out.size() == (int)joint_names.size())
            rpf->max_(0) = max_ctrl_out(i);
        rpf_.push_back(rpf);
    }

    ctrl_output_.resize(joint_names.size());
    ctrl_output_.names = joint_names;
    ctrl_error_.resize(joint_names.size());
    ctrl_error_.names = joint_names;
    activation_.resize(joint_names.size());
    activation_.setZero();

    return true;
}

bool JointRepPotField::startHook()
{
    if (! JointRepPotFieldBase::startHook())
        return false;

    return true;
}

void JointRepPotField::updateHook()
{
    JointRepPotFieldBase::updateHook();

    if(_feedback.read(feedback_) == RTT::NoData){
        if((base::Time::now() - stamp_).toSeconds() > 1)
        {
            LOG_DEBUG("No data on feedback port");
            stamp_ = base::Time::now();
        }
        return;
    }

    for(uint i = 0; i < rpf_.size(); i++){
        uint idx;
        try{
            idx = feedback_.mapNameToIndex(ctrl_output_.names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("No such joint name in feedback vector: %s", ctrl_output_.names[i].c_str());
            throw std::invalid_argument("Invalid input vector");
        }
        rpf_[i]->q_(0) = feedback_[idx].position;
        rpf_[i]->update();
        ctrl_output_[i].speed = rpf_[i]->ctrl_out_(0);
        ctrl_output_[i].effort = rpf_[i]->ctrl_out_(0);
        ctrl_error_[i].speed = rpf_[i]->q0_(0) - rpf_[i]->q_(0);
        activation_(i) = rpf_[i]->activation_;
    }

    _activation.write(activation_);
    _control_error.write(ctrl_error_);
    _ctrl_out.write(ctrl_output_);

}

void JointRepPotField::cleanupHook()
{
    JointRepPotFieldBase::cleanupHook();
    for(uint i = 0; i < rpf_.size(); i++)
        delete rpf_[i];
    rpf_.clear();
    ctrl_output_.clear();
    ctrl_error_.clear();
}
