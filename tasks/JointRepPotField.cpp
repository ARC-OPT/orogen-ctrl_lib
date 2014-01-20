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

    base::samples::Joints q_zero = _q_zero.get();
    base::samples::Joints d_zero = _d_zero.get();
    base::VectorXd kp = _kp.get();

    if(q_zero.size() != d_zero.size()){
        LOG_ERROR("q_zero property has size %i and d_zero property size %i", q_zero.size(), d_zero.size());
        return false;
    }
    if(kp.rows() != q_zero.size()){
        LOG_ERROR("q_zero property has size %i and kp property size %i", q_zero.size(), kp.size());
        return false;
    }

    for(uint i = 0; i < q_zero.size(); i++){
        RepulsivePotentialField* rpf = new RepulsivePotentialField(1);
        rpf->d0_ = d_zero[i].position;
        rpf->q0_(0) = q_zero[i].position;
        rpf->kp_(0) = kp(i);
        rpf_.push_back(rpf);
    }

    ctrl_output_.resize(q_zero.size());
    ctrl_output_.names = q_zero.names;

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
        LOG_DEBUG("No data on feedback port");
        return;
    }

    if(rpf_.size() != feedback_.size()){
        LOG_ERROR("Feedback has size %i but should have size %i", feedback_.size(), rpf_.size());
        throw std::invalid_argument("Invalid input size");
    }

    for(uint i = 0; i < rpf_.size(); i++){
        rpf_[i]->q_(0) = feedback_[i].position;
        rpf_[i]->update();
        ctrl_output_[i].speed = rpf_[i]->gradU_(0);
    }

    _ctrl_out.write(ctrl_output_);

}

void JointRepPotField::cleanupHook()
{
    JointRepPotFieldBase::cleanupHook();
    for(uint i = 0; i < rpf_.size(); i++)
        delete rpf_[i];
    rpf_.clear();
}
