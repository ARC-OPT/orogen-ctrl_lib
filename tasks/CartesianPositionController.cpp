/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <base-logging/Logging.hpp>
#include <ctrl_lib/ProportionalFeedForwardController.hpp>

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

    controller = new ProportionalFeedForwardController(_field_names.get().size());
    controller->setPropGain(_prop_gain.get());
    controller->setFeedforwardGain(_ff_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    return true;
}

bool CartesianPositionController::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;
    controller->clearSetpoint();
    controller->clearFeedback();
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
    RTT::FlowStatus fs = _setpoint.readNewest(setpoint);
    if(fs == RTT::NewData){
        setpoint.targetFrame = feedback.targetFrame;
        setpoint.sourceFrame = feedback.sourceFrame + "_setpoint";
        _current_setpoint.write(setpoint);
    }
    if(fs != RTT::NoData)
        setControlInput();
    return controller->hasSetpoint();
}

const base::VectorXd& CartesianPositionController::updateController(){
    return controller->update();
}

const base::VectorXd& CartesianPositionController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    for(uint i = 0; i < controller->getDimension(); i++)
        tmp(i) = fabs(controller->getControlOutput()(i))/controller->getMaxControlOutput()(i);
    return activation_function.compute(tmp);
}

void CartesianPositionController::writeControlOutput(const base::VectorXd &ctrl_output_raw){
    control_output.velocity = ctrl_output_raw.segment(0,3);
    control_output.angular_velocity = ctrl_output_raw.segment(3,3);
    control_output.sourceFrame = feedback.sourceFrame + "_setpoint";
    control_output.targetFrame = feedback.targetFrame;
    control_output.time = base::Time::now();
    _control_output.write(control_output);
    _control_error.write(controller->getControlError());
}

void CartesianPositionController::setControlInput(){

    setpoint_raw.resize(6);
    feedback_raw.resize(6);
    feedforward_raw.resize(6);

    if(!setpoint.hasValidPosition() || !setpoint.hasValidOrientation()){
        LOG_ERROR("Invalid setpoint. Either NaN or invalid orientation quaternion.");
        throw std::invalid_argument("Invalid setpoint");
    }
    if(!feedback.hasValidPosition() || !feedback.hasValidOrientation()){
        LOG_ERROR("Invalid feedback. Either NaN or invalid orientation quaternion.");
        throw std::invalid_argument("Invalid feedback");
    }

    // Set reference value to control error and actual value to zero, since the controller
    // cannot deal with full poses. Use pose_diff to compute the orientation-error
    // as zyx-rotation, since the controller cannot deal with full poses. This will give the
    // rotational velocity in 3D space that rotates the actual pose (feedback) onto the setpoint
    pose_diff(feedback, setpoint, 1, setpoint_raw);
    feedback_raw.setZero();

    // Set feedforward to given velocity setpoint. Set to zero if no valid velocitiy is given.
    feedforward_raw.setZero();
    if(setpoint.hasValidVelocity() && setpoint.hasValidAngularVelocity()){
        feedforward_raw.segment(0,3) = setpoint.velocity;
        feedforward_raw.segment(3,3) = setpoint.angular_velocity;
    }

    controller->setFeedback(feedback_raw);
    controller->setSetpoint(setpoint_raw);
    controller->setFeedforward(feedforward_raw);
}
