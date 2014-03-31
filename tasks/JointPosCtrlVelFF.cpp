/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPosCtrlVelFF.hpp"
#include <base/logging.h>

using namespace ctrl_lib;

JointPosCtrlVelFF::JointPosCtrlVelFF(std::string const& name)
    : JointPosCtrlVelFFBase(name)
{
}

JointPosCtrlVelFF::JointPosCtrlVelFF(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPosCtrlVelFFBase(name, engine)
{
}

bool JointPosCtrlVelFF::configureHook()
{
    if (! JointPosCtrlVelFFBase::configureHook())
        return false;

    joint_names_ = _joint_names.get();
    kp_ = _kp.get();
    kd_ = _kd.get();
    max_ctrl_out_ = _max_ctrl_out.get();

    if(joint_names_.size() != kp_.size())
    {
        LOG_ERROR("Kp property should have size %i but has size %i", joint_names_.size(), kp_.size());
        return false;
    }
    if(joint_names_.size() != kd_.size())
    {
        LOG_ERROR("Kd property should have size %i but has size %i", joint_names_.size(), kd_.size());
        return false;
    }

    if(max_ctrl_out_.size() == 0){
        max_ctrl_out_.resize(joint_names_.size());
        max_ctrl_out_.setConstant(base::infinity<double>());
    }

    if(max_ctrl_out_.size() != joint_names_.size()){
        LOG_ERROR("Max Ctrl out property should have size %i but has size %i", joint_names_.size(), max_ctrl_out_.size());
        return  false;
    }

    ctrl_output_.resize(joint_names_.size());
    ctrl_output_.names = joint_names_;
    ctrl_error_.resize(joint_names_.size());
    ctrl_error_.names = joint_names_;
    ctrl_out_.resize(joint_names_.size());
    x_r_.resize(joint_names_.size());
    x_.resize(joint_names_.size());
    v_r_.resize(joint_names_.size());

    return true;
}

bool JointPosCtrlVelFF::startHook()
{
    if (! JointPosCtrlVelFFBase::startHook())
        return false;

    ctrl_out_.setZero();
    v_r_.setZero();
    x_.setZero();
    x_r_.setZero();

    return true;
}

void JointPosCtrlVelFF::updateHook()
{
    JointPosCtrlVelFFBase::updateHook();

    if(_setpoint.read(ref_) == RTT::NoData)
    {
        if((base::Time::now() - stamp_).toSeconds() > 2.0)
        {
            LOG_DEBUG("%s: No data on set point port", this->getName().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }

    if(_feedback.read(cur_) == RTT::NoData)
    {
        if((base::Time::now() - stamp_).toSeconds() > 1.0)
        {
            LOG_DEBUG("%s: No data on feedback point port");
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
            LOG_ERROR("%s: Joint %s has invalid feedback position", this->getName().c_str(), joint_names_[i].c_str());
            throw std::invalid_argument("Invalid feedback input");
        }

        x_(i) = cur_[idx].position;

        try{
            idx = ref_.mapNameToIndex(joint_names_[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Name %s if not in reference vector", joint_names_[i].c_str());
            throw std::invalid_argument("Invalid reference input");
        }

        if(!ref_[idx].hasPosition())
        {
            LOG_ERROR("%s: Joint %s has invalid reference position", this->getName().c_str(), joint_names_[i].c_str());
            throw std::invalid_argument("Invalid reference input");
        }

        x_r_(i) = ref_[idx].position;

        if(ref_[idx].hasSpeed())
            v_r_(i) = ref_[idx].speed;
    }

    //Read new pid values
    if(_kp_values.read(kp_) == RTT::NewData){
        if(!kp_.size() != (int)joint_names_.size())
        {
            LOG_ERROR("Kp Values should have size %i but have size %i", joint_names_.size(), kp_.size());
            throw std::invalid_argument("Invalid gain input");
        }
    }

    //Control law:
    ctrl_out_ = kd_.cwiseProduct(v_r_) + kp_.cwiseProduct(x_r_ - x_);

    //Apply saturation: ctrl_out_ = ctrl_out_ * min(1, v_max/|ctrl_out_|). Scale all entries of ctrl_out_ appriopriately.
    double eta = 1;
    for(uint i = 0; i < joint_names_.size(); i++){
        if(ctrl_out_(i) != 0)
            eta = std::min( eta, max_ctrl_out_(i)/fabs(ctrl_out_(i)) );
    }
    ctrl_out_ = ctrl_out_ * eta;


    for(uint i = 0; i < ctrl_output_.size(); i++){
        ctrl_output_[i].speed = ctrl_out_(i);
        ctrl_error_[i].speed = x_r_(i) - x_(i);
    }

    ctrl_output_.time = base::Time::now();
    ctrl_error_.time = base::Time::now();
    _ctrl_out.write(ctrl_output_);
    _control_error.write(ctrl_error_);
}

void JointPosCtrlVelFF::cleanupHook()
{
    JointPosCtrlVelFFBase::cleanupHook();
}
