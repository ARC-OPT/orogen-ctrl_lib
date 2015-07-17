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
    jointNames = _jointNames.get();
    controller = new PositionControlFeedForward(jointNames.size());
    controlOutput.resize(jointNames.size());
    controlOutput.names = jointNames;

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
    if(_setpoint.read(setpoint) == RTT::NoData)
        return false;
    else{
        extractPositions(setpoint, jointNames, ((PositionControlFeedForward*)controller)->xr);
        extractSpeeds(setpoint, jointNames, ((PositionControlFeedForward*)controller)->vr);
        return true;
    }
}

bool JointPositionController::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        extractPositions(feedback, jointNames, ((PositionControlFeedForward*)controller)->x);
        return true;
    }
}

void JointPositionController::writeControlOutput(const Eigen::VectorXd &y){
    for(size_t i = 0; i < jointNames.size(); i++)
        controlOutput[i].speed = y(i);
    controlOutput.time = base::Time::now();
    _controlOutput.write(controlOutput);
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
