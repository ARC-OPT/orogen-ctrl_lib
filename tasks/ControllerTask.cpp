/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTask.hpp"
#include <ctrl_lib/Controller.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

ControllerTask::ControllerTask(std::string const& name)
    : ControllerTaskBase(name),
      controller(0)
{
}

ControllerTask::ControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ControllerTaskBase(name, engine),
      controller(0)
{
}

ControllerTask::~ControllerTask()
{
}

bool ControllerTask::configureHook()
{
    if (! ControllerTaskBase::configureHook())
        return false;

    field_names = _field_names.get();
    activation_function = _activation_function.get();

    return true;
}
bool ControllerTask::startHook()
{
    if (! ControllerTaskBase::startHook())
        return false;
    return true;
}

void ControllerTask::updateHook()
{
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

    controller->update(control_output_raw);
    writeControlOutput(control_output_raw);
    writeActivationFunction();
}

void ControllerTask::errorHook()
{
    ControllerTaskBase::errorHook();
}
void ControllerTask::stopHook()
{
    ControllerTaskBase::stopHook();
}
void ControllerTask::cleanupHook()
{
    ControllerTaskBase::cleanupHook();
}
