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

bool JointPositionController::configureHook(){
    if (! JointPositionControllerBase::configureHook())
        return false;

    controller = new ProportionalFeedForwardController(_field_names.get().size());
    controller->setPropGain(_prop_gain.get());
    controller->setFeedforwardGain(_ff_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    return true;
}

bool JointPositionController::startHook(){
    if (! JointPositionControllerBase::startHook())
        return false;
    controller->clearSetpoint();
    controller->clearFeedback();
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
    if(_feedback.readNewest(feedback) == RTT::NewData){
        extractPositions(feedback, field_names, feedback_raw);
        controller->setFeedback(feedback_raw);
        _current_feedback.write(feedback);
    }
    return controller->hasFeedback();
}

bool JointPositionController::readSetpoint(){

    if(_setpoint.readNewest(setpoint) == RTT::NewData){
        extractPositions(setpoint, field_names, setpoint_raw);
        extractVelocities(setpoint, field_names, feedforward_raw);

        controller->setSetpoint(setpoint_raw);
        controller->setFeedforward(feedforward_raw);
        _current_setpoint.write(setpoint);
    }
    return controller->hasSetpoint();
}

const base::VectorXd& JointPositionController::updateController(){
    return controller->update();
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
    _control_error.write(controller->getControlError());
}

const base::VectorXd& JointPositionController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    for(uint i = 0; i < controller->getDimension(); i++)
        tmp(i) = fabs(controller->getControlOutput()(i))/controller->getMaxControlOutput()(i);
    return activation_function.compute(tmp);
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
