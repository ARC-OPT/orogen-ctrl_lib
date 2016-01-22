/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTask.hpp"
#include <ctrl_lib/Controller.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

ControllerTask::ControllerTask(std::string const& name)
    : ControllerTaskBase(name),
      controller(0){
}

ControllerTask::ControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ControllerTaskBase(name, engine),
      controller(0){
}

ControllerTask::~ControllerTask(){
}

bool ControllerTask::configureHook(){
    if (! ControllerTaskBase::configureHook())
        return false;

    field_names = _field_names.get();
    controller->setMaxControlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());
    controller->setPropGain(_prop_gain.get());
    controller->setActivationFunction(_activation_function.get());

    return true;
}

bool ControllerTask::startHook(){
    if (! ControllerTaskBase::startHook())
        return false;

    controller->clearFeedback();
    controller->clearSetpoint();

    return true;
}

void ControllerTask::updateHook(){
    ControllerTaskBase::updateHook();

    controller->setMaxControlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());
    controller->setPropGain(_prop_gain.get());

    _current_max_control_output.write(controller->getMaxControlOutput());
    _current_dead_zone.write(controller->getDeadZone());
    _current_prop_gain.write(controller->getPropGain());

    if(!readFeedback()){
        if(state() != NO_FEEDBACK)
            state(NO_FEEDBACK);
        return;
    }
    if(!readSetpoint()){
        if(state() != NO_SETPOINT)
            state(NO_SETPOINT);
        return;
    }
    if(state() != RUNNING)
        state(RUNNING);

    writeControlOutput(controller->update());
    controller->evaluate();
    _activation.write(controller->computeActivation());
    _control_error.write(controller->getControlError());
    _ctrl_performance.write(controller->getControllerPerformance());
}

void ControllerTask::errorHook(){
    ControllerTaskBase::errorHook();
}

void ControllerTask::stopHook(){
    ControllerTaskBase::stopHook();
}

void ControllerTask::cleanupHook(){
    ControllerTaskBase::cleanupHook();
}

void ControllerTask::startEvaluation(bool start){
    if(controller){
        if(start)
            controller->resetEvaluation();
        controller->startEvaluation(start);
    }
}
