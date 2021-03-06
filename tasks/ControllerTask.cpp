/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTask.hpp"

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

    return true;
}

bool ControllerTask::startHook(){
    if (! ControllerTaskBase::startHook())
        return false;

    return true;
}

void ControllerTask::updateHook(){
    base::Time now = base::Time::now();
    if(!stamp.isNull())
        _actual_cycle_time.write((now-stamp).toSeconds());
    stamp = now;

    ControllerTaskBase::updateHook();

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

    updateController();
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
