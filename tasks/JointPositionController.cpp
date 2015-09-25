/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPositionController.hpp"
#include <ctrl_lib/PositionControlFeedForward.hpp>

using namespace ctrl_lib;

JointPositionController::JointPositionController(std::string const& name)
    : JointPositionControllerBase(name){
}

JointPositionController::JointPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPositionControllerBase(name, engine){
}

JointPositionController::~JointPositionController(){
}

bool JointPositionController::configureHook(){

    field_names = _field_names.get();
    controller = new PositionControlFeedForward(field_names.size());
    control_output.resize(field_names.size());
    control_output.names = field_names;

    if (! JointPositionControllerBase::configureHook())
        return false;

    return true;
}

bool JointPositionController::startHook(){
    if (! JointPositionControllerBase::startHook())
        return false;
    return true;
}

bool JointPositionController::readSetpoints(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{
        extractPositions(setpoint, field_names, controller->setpoint);
        extractSpeeds(setpoint, field_names, ((PositionControlFeedForward*)controller)->feed_forward_vel);
        return true;
    }
}

bool JointPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        extractPositions(feedback, field_names, controller->feedback);
        return true;
    }
}

void JointPositionController::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = ctrl_output_raw(i);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
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
