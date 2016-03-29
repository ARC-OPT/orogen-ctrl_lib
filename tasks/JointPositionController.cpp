/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPositionController.hpp"
#include <base/Logging.hpp>
#include <ctrl_lib/ProportionalFeedForwardController.hpp>

using namespace ctrl_lib;

JointPositionController::JointPositionController(std::string const& name)
    : JointPositionControllerBase(name){
}

JointPositionController::JointPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPositionControllerBase(name, engine){
}

bool JointPositionController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NewData){
        extractPositions(setpoint, field_names, setpoint_raw);
        extractVelocities(setpoint, field_names, feedforward_raw);

        ProportionalFeedForwardController* ctrl = (ProportionalFeedForwardController*)controller;
        ctrl->setSetpoint(setpoint_raw);
        ctrl->setFeedforward(feedforward_raw);

        _current_setpoint.write(setpoint);
    }
    return controller->hasSetpoint();
}

bool JointPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NewData){
        extractPositions(feedback, field_names, feedback_raw);
        controller->setFeedback(feedback_raw);
        _current_feedback.write(feedback);
    }
    return controller->hasFeedback();
}

void JointPositionController::writeControlOutput(const base::VectorXd& control_output_raw){
    if(control_output.size() != controller->getDimension()){
        control_output.resize(field_names.size());
        control_output.names = field_names;
    }
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = control_output_raw(i);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void JointPositionController::reset(){
    if(controller->hasFeedback()){
        setpoint.resize(field_names.size());
        setpoint.names = field_names;
        for(size_t i = 0; i < field_names.size(); i++){
            const base::JointState& state = feedback.getElementByName(field_names[i]);
            setpoint[i].position = state.position;
        }
        _current_setpoint.write(setpoint);
        extractPositions(setpoint, field_names, setpoint_raw);
        controller->setSetpoint(setpoint_raw);
        feedforward_raw.resize(controller->getDimension(), 0);
        controller->setFeedforward(feedforward_raw);
    }
}

void JointPositionController::extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& positions){
    positions.resize(names.size());
    for(size_t i = 0; i < names.size(); i++){
        const base::JointState& elem = joints.getElementByName(names[i]);
        if(!elem.hasPosition()){
            LOG_ERROR("%s: Element %s does not have a valid position value", this->getName().c_str(), names[i].c_str());
            throw std::invalid_argument("Invalid joints vector");
        }
        positions(i) = elem.position;
    }
}

void JointPositionController::extractVelocities(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& velocities){
    velocities.resize(names.size());
    velocities.setZero();
    for(size_t i = 0; i < names.size(); i++){
        const base::JointState& elem = joints.getElementByName(names[i]);
        if(!elem.hasSpeed()){ // If no speeds are given for an element, disable feed forward term
            velocities.setConstant(0);
            return;
        }
        velocities(i) = elem.speed;
    }
}
