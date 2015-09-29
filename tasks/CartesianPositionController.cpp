/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <ctrl_lib/ProportionalController.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

CartesianPositionController::CartesianPositionController(std::string const& name)
    : CartesianPositionControllerBase(name){
}

CartesianPositionController::CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianPositionControllerBase(name, engine){
}

CartesianPositionController::~CartesianPositionController(){
}

bool CartesianPositionController::configureHook(){

    setpoint.invalidatePosition();
    setpoint.invalidateOrientation();
    setpoint.invalidateVelocity();
    setpoint.invalidateAngularVelocity();

    setpoint = _initial_setpoint.get();

    if (! CartesianPositionControllerBase::configureHook())
        return false;

    return true;
}

bool CartesianPositionController::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;

    return true;
}

bool CartesianPositionController::readSetpoint(){

    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{

        if(!setpoint.hasValidPosition() || !setpoint.hasValidOrientation()){
            LOG_ERROR("%s: Setpoint does not have a valid position or orientation value", this->getName().c_str());
            throw std::invalid_argument("Invalid setpoint");
        }

        // Compute orientation error as quaternion
        orientation_error = setpoint.orientation * feedback.orientation.inverse();

        // Set reference value to control error and actual value to zero, since the controller
        // cannot deal with full poses. Use angle-axis representation to compute the orientation-error
        // as xyz-rotation, since the controller cannot deal with full poses. This will give the
        // rotational velocity in 3D space that rotates the actual pose (feedback) onto the setpoint
        controller->setpoint.resize(6);
        controller->setpoint.segment(0,3) = setpoint.position - feedback.position;
        controller->setpoint.segment(3,3) = orientation_error.axis()* orientation_error.angle();
        controller->feedback.resize(6);
        controller->feedback.setZero();

        return true;
    }
}

bool CartesianPositionController::readFeedback(){

    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{

        if(!feedback.hasValidPosition() || !feedback.hasValidOrientation()){
            LOG_ERROR("%s: Feedback term does not have a valid position or orientation value", this->getName().c_str());
            throw std::invalid_argument("Invalid feedback term");
        }

        _current_feedback.write(feedback);
        return true;
    }
}

void CartesianPositionController::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    control_output.velocity = ctrl_output_raw.segment(0,3);
    control_output.angular_velocity = ctrl_output_raw.segment(3,3);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
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
}
