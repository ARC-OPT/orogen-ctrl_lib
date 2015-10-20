/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianForceController.hpp"
#include <base/Logging.hpp>

using namespace ctrl_lib;

CartesianForceController::CartesianForceController(std::string const& name)
    : CartesianForceControllerBase(name)
{
}

CartesianForceController::CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianForceControllerBase(name, engine)
{
}

CartesianForceController::~CartesianForceController()
{
}

bool CartesianForceController::configureHook()
{
    if (! CartesianForceControllerBase::configureHook())
        return false;

    return true;
}
bool CartesianForceController::startHook()
{
    if (! CartesianForceControllerBase::startHook())
        return false;

    invalidate(setpoint);
    invalidate(feedback);

    return true;
}

bool CartesianForceController::readFeedback(){

    if(_feedback.readNewest(feedback) == RTT::NewData)
        has_feedback = true;

    if(has_feedback){

        if(!isValid(feedback)){
            LOG_ERROR("%s: Feedback has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid feedback term");
        }

        _current_feedback.write(feedback);
        ProportionalController* ctrl = (ProportionalController*)controller;
        ctrl->feedback.segment(0,3) = feedback.force;
        ctrl->feedback.segment(3,3) = feedback.force;
        return true;
    }
    else
        return false;
}

bool CartesianForceController::readSetpoint(){

    if(_feedback.readNewest(setpoint) == RTT::NewData)
        has_setpoint = true;

    if(has_setpoint){

        if(!isValid(setpoint)){
            LOG_ERROR("%s: Setpoint has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid setpoint");
        }

        ProportionalController* ctrl = (ProportionalController*)controller;
        ctrl->setpoint.segment(0,3) = setpoint.force;
        ctrl->setpoint.segment(3,3) = setpoint.force;
        _current_setpoint.write(setpoint);
        return true;
    }
    else
        return false;
}

void CartesianForceController::writeControlOutput(const base::VectorXd &ctrl_output_raw){
    control_output.velocity = ctrl_output_raw.segment(0,3);
    control_output.angular_velocity = ctrl_output_raw.segment(3,3);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianForceController::updateHook()
{
    CartesianForceControllerBase::updateHook();
}

void CartesianForceController::errorHook()
{
    CartesianForceControllerBase::errorHook();
}
void CartesianForceController::stopHook()
{
    CartesianForceControllerBase::stopHook();
}
void CartesianForceController::cleanupHook()
{
    CartesianForceControllerBase::cleanupHook();
}

void CartesianForceController::reset(){
    if(has_feedback){
        setpoint = feedback;
        has_setpoint = true;
    }
}
