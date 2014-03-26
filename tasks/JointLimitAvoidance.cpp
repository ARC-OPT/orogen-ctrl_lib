/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointLimitAvoidance.hpp"
#include <base/logging.h>

using namespace ctrl_lib;

JointLimitAvoidance::JointLimitAvoidance(std::string const& name)
    : JointLimitAvoidanceBase(name)
{
}

JointLimitAvoidance::JointLimitAvoidance(std::string const& name, RTT::ExecutionEngine* engine)
    : JointLimitAvoidanceBase(name, engine)
{
}

JointLimitAvoidance::~JointLimitAvoidance()
{
}

bool JointLimitAvoidance::configureHook()
{
    if (! JointLimitAvoidanceBase::configureHook())
        return false;

    limits_ = _joint_limits.get();
    d_zero_ = _d_zero.get();
    transition_range_ = _transition_range.get();
    max_ctrl_out_ = _max_ctrl_out.get();
    kp_ = _kp.get();

    if(max_ctrl_out_.size() != (int)limits_.size()){
        LOG_ERROR("Max ctrl output should have size %i but has size %i", limits_.size(), max_ctrl_out_.size());
        return false;
    }
    if(d_zero_.size() != (int)limits_.size()){
        LOG_ERROR("Joint Limits have size %i and d_zero has size %i", limits_.size(), d_zero_.size());
        return false;
    }
    if(kp_.size() != (int)limits_.size()){
        LOG_ERROR("Joint Limits have size %i and kp has size %i", limits_.size(), kp_.size());
        return false;
    }

    if(transition_range_ < 0 || transition_range_ > 1){
        LOG_ERROR("Transition range must be within 0 .. 1");
        return false;
    }

    ctrl_output_.resize(limits_.size());
    ctrl_output_.names = limits_.names;
    ctrl_error_.resize(limits_.size());
    ctrl_error_.names = limits_.names;
    activation_.resize(limits_.size());

    return true;
}

bool JointLimitAvoidance::startHook()
{
    if (! JointLimitAvoidanceBase::startHook())
        return false;

    for(size_t i = 0; i < ctrl_output_.size(); i++)
        ctrl_output_[i].speed = ctrl_output_[i].effort = 0;
    for(size_t i = 0; i < ctrl_error_.size(); i++)
        ctrl_error_[i].position = 0;
    activation_.setZero();

    stamp_ = base::Time::now();
    return true;
}

void JointLimitAvoidance::updateHook()
{
    JointLimitAvoidanceBase::updateHook();

    if(_feedback.read(feedback_) == RTT::NoData){
        if((base::Time::now() - stamp_).toSeconds() > 1)
        {
            LOG_DEBUG("No data on feedback port");
            stamp_ = base::Time::now();
        }
        return;
    }

    for(uint i = 0; i < limits_.size(); i++){

        base::JointLimitRange range = limits_[i];
        double position;

        try{
            position = feedback_.getElementByName(limits_.names[i]).position;
        }
        catch(std::exception e){
            LOG_DEBUG("No such joint name in feedback vector: %s. Skipping it.", limits_.names[i].c_str());
            continue;
        }

        double d_upper = fabs(range.max.position - position);
        double d_lower = fabs(range.min.position - position);
        double d0 = d_zero_(i);
        ctrl_output_[i].speed = 0;
        activation_(i) = 0;

        if(d_upper <= d_lower){
            //activation zone:
            if(d_upper < d0)
            {
                //Control output: Proportional to 1/d and limited by max_ctrl_out
                ctrl_output_[i].speed = std::max(-kp_(i) * (d_zero_(i) - d_upper)*(d_zero_(i) - d_upper), -max_ctrl_out_(i));

                //Activation function: piecewise linear profile
                if(d_upper < (1-transition_range_)*d0)
                    activation_(i) = 1;
                else
                    activation_(i) = ((d0 - d_upper)*(d0 - d_upper))/((transition_range_*d0)*(transition_range_*d0));

                LOG_DEBUG("Joint %s within lower activation zone. d_lower: %f, ctrl_out: %f, limited_ctrl_out: %f activation: %f",
                          limits_.names[i].c_str(), d_upper, -kp_(i) * (d_zero_(i) - d_upper), ctrl_output_[i].speed, activation_(i));
            }
            ctrl_error_[i].position = range.max.position - position;
        }
        else{
            //activation zone:
            if(d_lower < d0)
            {
                //Control output: Proportional to d0-d and limited by max_ctrl_out
                ctrl_output_[i].speed = std::min(kp_(i) * (d_zero_(i) - d_lower)*(d_zero_(i) - d_lower), max_ctrl_out_(i));

                //Activation function: piecewise quadratic profile
                if(d_lower < (1-transition_range_)*d0)
                    activation_(i) = 1;
                else
                    activation_(i) = ((d0 - d_lower)*(d0 - d_lower))/((transition_range_*d0)*(transition_range_*d0));

                LOG_DEBUG("Joint %s within lower activation zone. d_lower: %f, ctrl_out: %f, limited_ctrl_out: %f activation: %f",
                          limits_.names[i].c_str(), d_lower, kp_(i) * (d_zero_(i) - d_lower), ctrl_output_[i].speed, activation_(i));
            }
            ctrl_error_[i].position = position - range.min.position;
        }
    }

    ctrl_output_.time = base::Time::now();
    ctrl_error_.time = base::Time::now();
    _activation.write(activation_);
    _control_error.write(ctrl_error_);
    _ctrl_out.write(ctrl_output_);
}

void JointLimitAvoidance::errorHook()
{
    JointLimitAvoidanceBase::errorHook();
}

void JointLimitAvoidance::stopHook()
{
    JointLimitAvoidanceBase::stopHook();
}
void JointLimitAvoidance::cleanupHook()
{
    JointLimitAvoidanceBase::cleanupHook();
}
