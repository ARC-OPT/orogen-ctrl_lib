/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianForceController.hpp"
#include <base/Logging.hpp>

using namespace ctrl_lib;

CartesianForceController::CartesianForceController(std::string const& name)
    : CartesianForceControllerBase(name){
}

CartesianForceController::CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianForceControllerBase(name, engine){
}

bool CartesianForceController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NewData){
        if(!isValid(feedback)){
            LOG_ERROR("%s: Feedback has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid feedback term");
        }
        controller->setFeedback(wrenchToRaw(feedback, feedback_raw));
        _current_feedback.write(feedback);
    }
    return controller->hasFeedback();
}

bool CartesianForceController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NewData){
        if(!isValid(setpoint)){
            LOG_ERROR("%s: Setpoint has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid setpoint");
        }
        controller->setSetpoint(wrenchToRaw(setpoint, setpoint_raw));
        _current_setpoint.write(setpoint);
    }
    return controller->hasSetpoint();
}

void CartesianForceController::writeControlOutput(const base::VectorXd &ctrl_output_raw){
    control_output.velocity = ctrl_output_raw.segment(0,3);
    control_output.angular_velocity = ctrl_output_raw.segment(3,3);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianForceController::clearSetpoint(){
    if(controller)
        controller->clearSetpoint();
}

void CartesianForceController::reset(){
    if(controller->hasFeedback()){
        setpoint = feedback;
        controller->setSetpoint(wrenchToRaw(setpoint, setpoint_raw));
        _current_setpoint.write(setpoint);
    }
}

bool CartesianForceController::isValid(const base::Wrench &w){
    return !base::isNaN(w.force(0)) && !base::isNaN(w.force(1)) && !base::isNaN(w.force(2)) &&
           !base::isNaN(w.torque(0)) && !base::isNaN(w.torque(1)) && !base::isNaN(w.torque(2));
}

void CartesianForceController::invalidate(base::Wrench& w){
    for(uint i = 0; i < 3; i++){
        w.force(i) = base::NaN<double>();
        w.torque(i) = base::NaN<double>();
    }
}

const base::VectorXd CartesianForceController::wrenchToRaw(const base::samples::Wrench& wrench, base::VectorXd& raw){
    raw.resize(6);
    raw.segment(0,3) = wrench.force;
    raw.segment(3,3) = wrench.torque;
    return raw;
}
