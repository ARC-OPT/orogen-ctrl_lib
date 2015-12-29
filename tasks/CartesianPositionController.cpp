/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <ctrl_lib/ProportionalFeedForwardController.hpp>

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

    controller = new ProportionalFeedForwardController(6);
    ((ProportionalFeedForwardController*)controller)->setFeedForwardGain(_ff_gain.get());

    if (! CartesianPositionControllerBase::configureHook())
        return false;

    return true;
}

bool CartesianPositionController::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;
    return true;
}

void CartesianPositionController::updateHook(){
    ProportionalFeedForwardController* ctrl = (ProportionalFeedForwardController*)controller;
    ctrl->setFeedForwardGain(_ff_gain.get());
    _current_ff_gain.write(ctrl->getFeedForwardGain());

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

bool CartesianPositionController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NewData)
        _current_setpoint.write(setpoint);

    if( ( setpoint.hasValidOrientation() && setpoint.hasValidPosition()) ||
        ( setpoint.hasValidVelocity()    && setpoint.hasValidAngularVelocity()) )
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

    // TODO: This ignores invalid input and can lead to silent errors! However, it may be useful
    // if we only want to use the feed forward term...
    if(!setpoint.hasValidPosition() || !setpoint.hasValidOrientation() ||
            !feedback.hasValidPosition() || !feedback.hasValidOrientation()){
        setpoint_raw.setZero();
        feedback_raw.setZero();
    }
    else{
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
    }

    // Set feedforward to given velocity setpoint. Set to zero if no valid velocitiy is given.
    if(setpoint.hasValidVelocity() && setpoint.hasValidAngularVelocity()){
        feedforward_raw.segment(0,3) = setpoint.velocity;
        feedforward_raw.segment(3,3) = setpoint.angular_velocity;
    }
    else
        feedforward_raw.setZero();

    ProportionalFeedForwardController* ctrl = (ProportionalFeedForwardController*)controller;
    ctrl->setFeedback(feedback_raw);
    ctrl->setSetpoint(setpoint_raw);
    ctrl->setFeedForward(feedforward_raw);
}

