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

    base::JointLimits limits = _joint_limits.get();
    base::VectorXd d_zero = _d_zero.get();
    double transition_range = _transition_range.get();
    base::VectorXd max_ctrl_out = _max_ctrl_out.get();
    base::VectorXd kp = _kp.get();


    if(d_zero.size() != (int)limits.size()){
        LOG_ERROR("Joint Limits have size %i and d_zero has size %i", limits.size(), d_zero.size());
        return false;
    }
    if(kp.size() != (int)limits.size()){
        LOG_ERROR("Joint Limits have size %i and kp has size %i", limits.size(), kp.size());
        return false;
    }

    if(transition_range < 0 || transition_range > 1){
        LOG_ERROR("Transition range must be within 0 .. 1");
        return false;
    }

    for(size_t i = 0; i < limits.size(); i++)
    {
        //Upper joint limit
        RepulsivePotentialField* upper_rpf = new RepulsivePotentialField(1);
        upper_rpf->d0_ = d_zero(i);
        upper_rpf->q0_(0) = limits[i].max.position;
        upper_rpf->kp_(0) = kp(i);
        if(max_ctrl_out.size() == (int)limits.size()) //If there is no max ctrl out given, it will be infinite
            upper_rpf->max_(0) = max_ctrl_out(i);
        upper_rpf->transition_range_ = transition_range;
        upper_rpf_.push_back(upper_rpf);

        //Lower joint limit
        RepulsivePotentialField* lower_rpf = new RepulsivePotentialField(1);
        lower_rpf->d0_ = d_zero(i);
        lower_rpf->q0_(0) = limits[i].min.position;
        lower_rpf->kp_(0) = kp(i);
        if(max_ctrl_out.size() == (int)limits.size()) //If there is no max ctrl out given, it will be infinite
            lower_rpf->max_(0) = max_ctrl_out(i);
        lower_rpf->transition_range_ = transition_range;
        lower_rpf_.push_back(lower_rpf);
    }

    ctrl_output_.resize(limits.size());
    ctrl_output_.names = limits.names;
    ctrl_error_.resize(limits.size());
    ctrl_error_.names = limits.names;
    activation_.resize(limits.size());

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

    for(uint i = 0; i < upper_rpf_.size(); i++){
        uint idx;
        try{
            idx = feedback_.mapNameToIndex(ctrl_output_.names[i]);
        }
        catch(std::exception e){
            LOG_DEBUG("No such joint name in feedback vector: %s. Skipping it.", ctrl_output_.names[i].c_str());
            continue;
        }
        upper_rpf_[i]->q_(0) = feedback_[idx].position;
        upper_rpf_[i]->update();

        lower_rpf_[i]->q_(0) = feedback_[idx].position;
        lower_rpf_[i]->update();

        RepulsivePotentialField *active_field;
        if(fabs(upper_rpf_[i]->ctrl_out_(0)) >= fabs(lower_rpf_[i]->ctrl_out_(0))){
            active_field = upper_rpf_[i];
        }
        else{
            active_field = lower_rpf_[i];
        }

        ctrl_output_[i].speed = ctrl_output_[i].effort = active_field->ctrl_out_(0);
        activation_(i) = active_field->activation_;

        double diff_upper = fabs(upper_rpf_[i]->q0_(0) - upper_rpf_[i]->q_(0));
        double diff_lower = fabs(lower_rpf_[i]->q0_(0) - lower_rpf_[i]->q_(0));
        if(diff_upper > diff_lower)
            ctrl_error_[i].position = diff_lower;
        else
            ctrl_error_[i].position = diff_upper;
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
