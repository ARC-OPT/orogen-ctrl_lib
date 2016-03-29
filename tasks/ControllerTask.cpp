/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTask.hpp"
#include <ctrl_lib/Controller.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

ControllerTask::ControllerTask(std::string const& name)
    : ControllerTaskBase(name){
}

ControllerTask::ControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ControllerTaskBase(name, engine){
}

bool ControllerTask::configureHook(){
    if (! ControllerTaskBase::configureHook())
        return false;

    field_names = _field_names.get();
    activation_function = _activation_function.get();
    activation.resize(_field_names.get().size());

    updateControllerProperties();

    return true;
}

bool ControllerTask::startHook(){
    if (! ControllerTaskBase::startHook())
        return false;

    return true;
}

void ControllerTask::updateHook(){
    ControllerTaskBase::updateHook();

    updateControllerProperties();

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

    writeControlOutput(updateController());
    _activation.write(computeActivation(activation_function));
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
