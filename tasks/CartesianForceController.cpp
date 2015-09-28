/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianForceController.hpp"

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

    setpoint = _initial_setpoint.get();

    return true;
}
bool CartesianForceController::startHook()
{
    if (! CartesianForceControllerBase::startHook())
        return false;
    return true;
}

bool CartesianForceController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        controller->feedback.segment(0,3) = feedback.force;
        controller->feedback.segment(3,3) = feedback.force;
        return true;
    }
}

bool CartesianForceController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{
        controller->setpoint.segment(0,3) = setpoint.force;
        controller->setpoint.segment(3,3) = setpoint.force;
        return true;
    }
}

void CartesianForceController::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
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
