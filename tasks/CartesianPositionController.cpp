/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <ctrl_lib/PositionControlFeedForward.hpp>

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
    //Cartesian position controller will have 6 dimensions (x,y,z,rotX,rotY,rotZ)
    controller = new PositionControlFeedForward(6);

    if (! CartesianPositionControllerBase::configureHook())
        return false;
    return true;
}

bool CartesianPositionController::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;
    return true;
}

bool CartesianPositionController::readSetpoints(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else
        return true;
}

bool CartesianPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        Eigen::VectorXd& xr = ((PositionControlFeedForward*)controller)->xr;
        Eigen::VectorXd& x = ((PositionControlFeedForward*)controller)->x;

        // Compute orientation error as quaternion
        orientationError = setpoint.orientation * feedback.orientation.inverse();

        // Set reference value to control error and actual value to zero, since the controller
        // cannot deal with full poses. Use angle-axis representation to compute the orientation-error
        // as xyz-rotation, since the controller cannot deal with full poses. This will give the
        // rotational velocity in 3D space that rotates the actual pose (feedback) onto the setpoint
        xr.resize(6);
        xr.segment(0,3) = setpoint.position - feedback.position;
        xr.segment(3,3) = orientationError.axis()* orientationError.angle();
        x.resize(6);
        x.setZero();
        return true;
    }
}

void CartesianPositionController::writeControlOutput(const Eigen::VectorXd &y){
    controlOutput.velocity = y.segment(0,3);
    controlOutput.angular_velocity = y.segment(3,3);
    controlOutput.time = base::Time::now();
    _controlOutput.write(controlOutput);
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
