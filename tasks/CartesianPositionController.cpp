/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>

using namespace ctrl_lib;

CartesianPositionController::CartesianPositionController(std::string const& name)
    : CartesianPositionControllerBase(name){
}

CartesianPositionController::CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianPositionControllerBase(name, engine){
}

bool CartesianPositionController::readSetpoint(){
    RTT::FlowStatus fs = _setpoint.readNewest(setpoint);
    if(fs == RTT::NewData)
        _current_setpoint.write(setpoint);
    if(fs != RTT::NoData)
        setControlInput();
    return controller->hasSetpoint();
}

bool CartesianPositionController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        return true;
    }
}

void CartesianPositionController::writeControlOutput(const base::VectorXd &ctrl_output_raw){
    control_output.velocity = ctrl_output_raw.segment(0,3);
    control_output.angular_velocity = ctrl_output_raw.segment(3,3);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianPositionController::reset(){
    if(feedback.hasValidPosition() && feedback.hasValidOrientation()){
        setpoint.position = feedback.position;
        setpoint.orientation = feedback.orientation;
        _current_setpoint.write(setpoint);
        setControlInput();
    }
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

    KDL::Frame setpoint_kdl, feedback_kdl;
    kdl_conversions::RigidBodyState2KDL(setpoint,setpoint_kdl);
    kdl_conversions::RigidBodyState2KDL(feedback,feedback_kdl);

    // Set reference value to control error and actual value to zero, since the controller
    // cannot deal with full poses. Use KDL::diff to compute the orientation-error
    // as zyx-rotation, since the controller cannot deal with full poses. This will give the
    // rotational velocity in 3D space that rotates the actual pose (feedback) onto the setpoint
    KDL::Twist diff = KDL::diff(feedback_kdl, setpoint_kdl);

    for(int i = 0; i < 6; i++)
        setpoint_raw(i) = diff(i);
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

