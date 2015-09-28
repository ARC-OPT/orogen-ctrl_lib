/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <ctrl_lib/ProportionalControllerFeedForward.hpp>
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

    //Cartesian position controller has to have 6 dimensions (x,y,z,rotX,rotY,rotZ)
    field_names = _field_names.get();
    if(field_names.size() != 6){
        LOG_ERROR("Size of field name vector should be 6, but is %i", field_names.size());
        return false;
    }
    controller = new ProportionalControllerFeedForward(field_names.size());

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
    _setpoint.readNewest(setpoint);
    if(!setpoint.hasValidPosition() || !setpoint.hasValidOrientation())
        return false;
    else{

        Eigen::VectorXd& xr = ((ProportionalControllerFeedForward* )controller)->setpoint;
        Eigen::VectorXd& x = ((ProportionalControllerFeedForward* )controller)->feedback;

        // Compute orientation error as quaternion
        orientation_error = setpoint.orientation * feedback.orientation.inverse();

        // Set reference value to control error and actual value to zero, since the controller
        // cannot deal with full poses. Use angle-axis representation to compute the orientation-error
        // as xyz-rotation, since the controller cannot deal with full poses. This will give the
        // rotational velocity in 3D space that rotates the actual pose (feedback) onto the setpoint
        xr.resize(6);
        xr.segment(0,3) = setpoint.position - feedback.position;
        xr.segment(3,3) = orientation_error.axis()* orientation_error.angle();
        x.resize(6);
        x.setZero();

        return true;
    }
}

bool CartesianPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        current_feedback.resize(6);
        current_feedback.segment(0,3) = feedback.position;
        current_feedback.segment(3,3) = base::getEuler(feedback.orientation);
        _current_feedback.write(current_feedback);
        return true;
    }
}

void CartesianPositionController::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    control_output.velocity = ctrl_output_raw.segment(0,3);
    control_output.angular_velocity = ctrl_output_raw.segment(3,3);
    control_output.time = base::Time::now();
    _control_output.write(control_output);

    _control_error.write(((ProportionalControllerFeedForward*)controller)->control_error);
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
    delete controller;
    CartesianPositionControllerBase::cleanupHook();
}
