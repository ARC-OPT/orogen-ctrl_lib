/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPositionController.hpp"
#include <base-logging/Logging.hpp>
#include <ctrl_lib/JointPosPDController.hpp>

using namespace ctrl_lib;

JointPositionController::JointPositionController(std::string const& name)
    : JointPositionControllerBase(name){
}

JointPositionController::JointPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPositionControllerBase(name, engine){
}

bool JointPositionController::configureHook(){
    if (! JointPositionControllerBase::configureHook())
        return false;

    controller = new JointPosPDController(_field_names.get());
    controller->setPGain(_p_gain.get());
    controller->setDGain(_d_gain.get());
    controller->setMaxCtrlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    return true;
}

bool JointPositionController::startHook(){
    if (! JointPositionControllerBase::startHook())
        return false;
    _setpoint.clear();
    _feedback.clear();
    return true;
}

void JointPositionController::updateHook(){
    JointPositionControllerBase::updateHook();
}

void JointPositionController::errorHook(){
    JointPositionControllerBase::errorHook();
}

void JointPositionController::stopHook(){
    JointPositionControllerBase::stopHook();
}

void JointPositionController::cleanupHook(){
    JointPositionControllerBase::cleanupHook();
    delete controller;
}

bool JointPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        return true;
    }
}

bool JointPositionController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{
        _current_setpoint.write(setpoint);
        return true;
    }
}

void JointPositionController::updateController(){
    control_output = controller->update(setpoint, feedback);

    _control_output.write(control_output);
    _control_error.write(controller->getControlError());
}

const base::VectorXd& JointPositionController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(control_output.size());
    for(uint i = 0; i < tmp.size(); i++)
        tmp(i) = fabs(control_output[i].speed)/controller->maxCtrlOutput()(i);
    return activation_function.compute(tmp);
}
