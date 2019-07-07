/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianForceController.hpp"
#include <base-logging/Logging.hpp>
#include <wbc/controllers/CartesianForcePIDController.hpp>

using namespace ctrl_lib;

CartesianForceController::CartesianForceController(std::string const& name)
    : CartesianForceControllerBase(name){
}

CartesianForceController::CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianForceControllerBase(name, engine){
}

bool CartesianForceController::configureHook(){
    if (! CartesianForceControllerBase::configureHook())
        return false;

    controller = new CartesianForcePIDController();
    PIDCtrlParams params(6);
    params.p_gain = _p_gain.get();
    controller->setPID(params);
    controller->setMaxCtrlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    return true;
}

bool CartesianForceController::startHook(){
    if (! CartesianForceControllerBase::startHook())
        return false;
    _setpoint.clear();
    _feedback.clear();
    return true;
}

void CartesianForceController::updateHook(){
    CartesianForceControllerBase::updateHook();
}

void CartesianForceController::errorHook(){
    CartesianForceControllerBase::errorHook();
}

void CartesianForceController::stopHook(){
    CartesianForceControllerBase::stopHook();
}

void CartesianForceController::cleanupHook(){
    CartesianForceControllerBase::cleanupHook();
    delete controller;
}

bool CartesianForceController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        if(!isValid(feedback)){
            LOG_ERROR("%s: Feedback has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid feedback term");
        }
        _current_feedback.write(feedback);
        return true;
    }
}

bool CartesianForceController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{
        if(!isValid(setpoint)){
            LOG_ERROR("%s: Setpoint has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid setpoint");
        }
        _current_setpoint.write(setpoint);
        return true;
    }
}

void CartesianForceController::updateController(){

    control_output = controller->update(setpoint, feedback, this->getActivity()->getPeriod());

    _control_output.write(control_output);
    _control_error.write(controller->getControlError());
}

const base::VectorXd& CartesianForceController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    for(uint i = 0; i < controller->getDimension(); i++)
        tmp(i) = fabs(ctrl_output_raw(i))/controller->maxCtrlOutput()(i);
    return activation_function.compute(tmp);
}
