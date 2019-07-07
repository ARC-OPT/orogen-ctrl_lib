/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointTorqueController.hpp"
#include <wbc/controllers/JointTorquePIDController.hpp>

using namespace ctrl_lib;

JointTorqueController::JointTorqueController(std::string const& name)
    : JointTorqueControllerBase(name)
{
}

JointTorqueController::JointTorqueController(std::string const& name, RTT::ExecutionEngine* engine)
    : JointTorqueControllerBase(name, engine){
}

bool JointTorqueController::configureHook(){
    if (! JointTorqueControllerBase::configureHook())
        return false;

    controller = new JointTorquePIDController(_field_names.get());
    controller->setPID(_pid_params.get());
    controller->setMaxCtrlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    return true;
}

bool JointTorqueController::startHook(){
    if (! JointTorqueControllerBase::startHook())
        return false;
    return true;
}

void JointTorqueController::updateHook(){
    JointTorqueControllerBase::updateHook();
}

void JointTorqueController::errorHook(){
    JointTorqueControllerBase::errorHook();
}

void JointTorqueController::stopHook(){
    JointTorqueControllerBase::stopHook();
}

void JointTorqueController::cleanupHook(){
    JointTorqueControllerBase::cleanupHook();
}

bool JointTorqueController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        return true;
    }
}

bool JointTorqueController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{
        _current_setpoint.write(setpoint);
        return true;
    }
}

void JointTorqueController::updateController(){
    control_output = controller->update(setpoint, feedback, this->getPeriod());

    _control_output.write(control_output);
    _control_error.write(controller->getControlError());
}

const base::VectorXd& JointTorqueController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(control_output.size());
    for(uint i = 0; i < tmp.size(); i++)
        tmp(i) = fabs(control_output[i].speed)/controller->maxCtrlOutput()(i);
    return activation_function.compute(tmp);
}
