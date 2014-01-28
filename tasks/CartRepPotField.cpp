/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartRepPotField.hpp"

using namespace ctrl_lib;

CartRepPotField::CartRepPotField(std::string const& name)
    : CartRepPotFieldBase(name)
{
    rpf_ = 0;
}

CartRepPotField::CartRepPotField(std::string const& name, RTT::ExecutionEngine* engine)
    : CartRepPotFieldBase(name, engine)
{
    rpf_ = 0;
}

bool CartRepPotField::configureHook()
{
    if (! CartRepPotFieldBase::configureHook())
        return false;

    rpf_ = new RepulsivePotentialField(3);
    rpf_->d0_ = _d_zero.get();

    std::vector<base::actuators::PIDValues> pid = _pid.get();
    if(pid.size() == 3){
        for(uint i = 0; i < 3; i++){
            rpf_->kp_(i) = pid[i].kp;
            rpf_->max_(i) = pid[i].maxPWM;
        }
    }

    ctrl_output_.angular_velocity.setZero();
    ctrl_output_.angular_acceleration.setZero();

    return true;
}

bool CartRepPotField::startHook()
{
    if (! CartRepPotFieldBase::startHook())
        return false;
    return true;
}

void CartRepPotField::updateHook()
{
    CartRepPotFieldBase::updateHook();

    //Get Transforms:
    _controlled_in2rep_field_center.get(base::Time::now(), center_);
    _controlled_in2controlled_frame.get(base::Time::now(), cur_);

    if(!center_.hasValidPosition()){
        LOG_DEBUG("Transform between controlled_in and rep_field_center has no valid position");
        return;
    }

    if(!cur_.hasValidPosition()){
        LOG_DEBUG("Transform between controlled_in and controlled_frame has no valid position");
        return;
    }

    rpf_->q0_ = center_.position;
    rpf_->q_ = cur_.position;

    rpf_->update();

    ctrl_output_.velocity = rpf_->ctrl_out_;
    ctrl_output_.acceleration = rpf_->ctrl_out_;
    _ctrl_out.write(ctrl_output_);

    //Write debug data
    ctrl_error_.velocity = (center_.position - cur_.position);
    _control_error.write(ctrl_error_);
}

void CartRepPotField::cleanupHook()
{
    CartRepPotFieldBase::cleanupHook();
    delete rpf_;
    rpf_ = 0;
}
