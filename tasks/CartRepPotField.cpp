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
    rpf_->kp_ = _kp.get();
    rpf_->max_ = _max_ctrl_out.get();
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
    _rep_field_center2controlled_in.get(base::Time::now(), center_);
    _controlled_frame2controlled_in.get(base::Time::now(), cur_);

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
    ctrl_error_.time = base::Time::now();
    _control_error.write(ctrl_error_);
}

void CartRepPotField::cleanupHook()
{
    CartRepPotFieldBase::cleanupHook();
    delete rpf_;
    rpf_ = 0;
}
