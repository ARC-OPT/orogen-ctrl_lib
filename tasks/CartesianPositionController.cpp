/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <base-logging/Logging.hpp>
#include <wbc/controllers/CartesianPosPDController.hpp>

using namespace ctrl_lib;

CartesianPositionController::CartesianPositionController(std::string const& name)
    : CartesianPositionControllerBase(name){
}

CartesianPositionController::CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianPositionControllerBase(name, engine){
}

bool CartesianPositionController::configureHook(){
    if (! CartesianPositionControllerBase::configureHook())
        return false;

    controller = new CartesianPosPDController();
    controller->setPGain(_p_gain.get());
    controller->setDGain(_d_gain.get());
    controller->setFFGain(_ff_gain.get());
    controller->setMaxCtrlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    return true;
}

bool CartesianPositionController::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;
    _setpoint.clear();
    _feedback.clear();
    return true;
}

void CartesianPositionController::updateHook(){
    CartesianPositionControllerBase::updateHook();
}

void CartesianPositionController::errorHook(){
    CartesianPositionControllerBase::errorHook();
}

void CartesianPositionController::stopHook(){
    CartesianPositionControllerBase::stopHook();
}

void CartesianPositionController::cleanupHook(){
    CartesianPositionControllerBase::cleanupHook();
    delete controller;
}

bool CartesianPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        return true;
    }
}

bool CartesianPositionController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{        
        _current_setpoint.write(setpoint);
        return true;
    }
}

void CartesianPositionController::updateController(){
    control_output = controller->update(setpoint, feedback);

    _control_output.write(control_output);
    _control_error.write(controller->getControlError());
}

const base::VectorXd& CartesianPositionController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(6);
    tmp.segment(0,3) = control_output.twist.linear;
    tmp.segment(3,3) = control_output.twist.angular;
    for(uint i = 0; i < 6; i++)
        tmp(i) = fabs(tmp(i))/controller->maxCtrlOutput()(i);
    return activation_function.compute(tmp);
}
