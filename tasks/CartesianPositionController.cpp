/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <ctrl_lib/ProportionalController.hpp>
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>

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

    if (! CartesianPositionControllerBase::configureHook())
        return false;

    return true;
}

bool CartesianPositionController::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;

    setpoint.invalidatePosition();
    setpoint.invalidateOrientation();
    setpoint.invalidateVelocity();
    setpoint.invalidateAngularVelocity();

    return true;
}

bool CartesianPositionController::readSetpoint(){

    if(_setpoint.readNewest(setpoint) == RTT::NewData){
        if(!setpoint.hasValidPosition() || !setpoint.hasValidOrientation()){
            LOG_ERROR("%s: Setpoint does not have a valid position or orientation value", this->getName().c_str());
            throw std::invalid_argument("Invalid setpoint");
        }
        _current_setpoint.write(setpoint);
        kdl_conversions::RigidBodyState2KDL(setpoint,setpoint_kdl);
        has_setpoint = true;
    }

    if(has_setpoint){

        // Set reference value to control error and actual value to zero, since the controller
        // cannot deal with full poses. Use KDL::diff to compute the orientation-error
        // as zyx-rotation, since the controller cannot deal with full poses. This will give the
        // rotational velocity in 3D space that rotates the actual pose (feedback) onto the setpoint
        KDL::Twist diff = KDL::diff(feedback_kdl, setpoint_kdl);

        ProportionalController* ctrl = (ProportionalController*)controller;

        ctrl->setpoint.resize(6);
        for(int i = 0; i < 6; i++)
            ctrl->setpoint(i) = diff(i);
        ctrl->feedback.resize(6);
        ctrl->feedback.setZero();

        if(setpoint.hasValidVelocity() && setpoint.hasValidAngularVelocity()){
            ctrl->feed_forward.segment(0,3) = setpoint.velocity;
            ctrl->feed_forward.segment(3,3) = setpoint.angular_velocity;
        }

        _current_setpoint.write(setpoint);

        return true;
    }
    else
        return false;
}

bool CartesianPositionController::readFeedback(){

    if(_feedback.readNewest(feedback) == RTT::NewData){

        if(!feedback.hasValidPosition() || !feedback.hasValidOrientation()){
            LOG_ERROR("%s: Feedback term does not have a valid position or orientation value", this->getName().c_str());
            throw std::invalid_argument("Invalid feedback term");
        }

        kdl_conversions::RigidBodyState2KDL(feedback,feedback_kdl);
        has_feedback = true;
        _current_feedback.write(feedback);
        return true;
    }
    return has_feedback;

}

void CartesianPositionController::writeControlOutput(const base::VectorXd &ctrl_output_raw){
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

void CartesianPositionController::reset(){
    if(has_feedback){
        setpoint.position = feedback.position;
        setpoint.orientation = feedback.orientation;
        has_setpoint = true;
    }
}
