/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianForceController.hpp"
#include <base-logging/Logging.hpp>
#include <wbc/controllers/CartesianForcePIDController.hpp>

using namespace ctrl_lib;

CartesianForceController::CartesianForceController(std::string const& name)
    : CartesianForceControllerBase(name){
}

CartesianForceController::CartesianForceController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianForceControllerBase(name, engine){
}

bool CartesianForceController::configureHook(){
    if (! CartesianForceControllerBase::configureHook())
        return false;

    controller = new wbc::CartesianForcePIDController();
    PIDCtrlParams params(controller->getDimension());
    params.p_gain = _p_gain.get();
    params.i_gain.setZero();
    params.d_gain.setZero();
    params.windup.setZero();
    controller->setPID(params);
    controller->setMaxCtrlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());
    ft_sensor_name = _ft_sensor_name.get();

    return true;
}

bool CartesianForceController::startHook(){
    if (! CartesianForceControllerBase::startHook())
        return false;
    _setpoint.clear();
    _feedback.clear();
    return true;
}

void CartesianForceController::updateHook(){
    CartesianForceControllerBase::updateHook();
}

void CartesianForceController::errorHook(){
    CartesianForceControllerBase::errorHook();
}

void CartesianForceController::stopHook(){
    CartesianForceControllerBase::stopHook();
}

void CartesianForceController::cleanupHook(){
    CartesianForceControllerBase::cleanupHook();
    delete controller;
}

bool CartesianForceController::readFeedback(){
    if(_feedback.readNewest(feedback) == RTT::NoData){
        if(_feedback_wrenches.readNewest(feedback_wrenches) == RTT::NoData)
            return false;
        else{
            try{
                feedback.time = feedback_wrenches.time;
                feedback.force = feedback_wrenches[ft_sensor_name].force;
                feedback.torque = feedback_wrenches[ft_sensor_name].torque;
            }
            catch(base::samples::Wrenches::InvalidName e){
                LOG_ERROR_S << "You configured FT Sensor name '" << ft_sensor_name << "', but this name is not included in wrenches feedbacl vector" << std::endl;
                throw e;
            }
        }
    }

    if(!isValid(feedback)){
        LOG_ERROR("%s: Feedback has an invalid force or torque value", this->getName().c_str());
        throw std::invalid_argument("Invalid feedback term");
    }
    _current_feedback.write(feedback);
    return true;
}

bool CartesianForceController::readSetpoint(){
    if(_setpoint.readNewest(setpoint) == RTT::NoData)
        return false;
    else{
        if(!isValid(setpoint)){
            LOG_ERROR("%s: Setpoint has an invalid force or torque value", this->getName().c_str());
            throw std::invalid_argument("Invalid setpoint");
        }
        _current_setpoint.write(setpoint);
        return true;
    }
}

void CartesianForceController::updateController(){

    control_output = controller->update(setpoint, feedback, this->getActivity()->getPeriod());

    _control_output.write(control_output);
    _control_error.write(controller->getControlError());
}

const base::VectorXd& CartesianForceController::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    for(uint i = 0; i < 3; i++){
        tmp(i) = fabs(control_output.twist.linear(i))/controller->maxCtrlOutput()(i);
        tmp(i+3) = fabs(control_output.twist.angular(i))/controller->maxCtrlOutput()(i+3);
    }
    return activation_function.compute(tmp);
}
