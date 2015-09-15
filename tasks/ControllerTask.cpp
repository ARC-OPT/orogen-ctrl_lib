/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTask.hpp"
#include <ctrl_lib/Controller.hpp>

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

    controller->kp = _propGain.get();
    if(_maxControlOutput.get().size() != 0)
        controller->yMax = _maxControlOutput.get();
    if(_deadZone.get().size() != 0)
        controller->eMin = _deadZone.get();

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

    _newPropGain.readNewest((base::VectorXd&)controller->kp);
    _newMaxControlOutput.readNewest((base::VectorXd&)controller->yMax);
    _newDeadZone.readNewest((base::VectorXd&)controller->eMin);

    _currentPropGain.write(controller->kp);
    _currentDeadZone.write(controller->eMin);
    _currentMaxControlOutput.write(controller->yMax);

    if(!readSetpoints()){
        if(state() != NO_SETPOINT)
            state(NO_SETPOINT);
    }
    else if(!readFeedback()){
        if(state() != NO_FEEDBACK)
            state(NO_FEEDBACK);
    }
    else{
        if(state() != RUNNING)
            state(RUNNING);

        controller->update(y);

        _controlError.write(controller->e);
        _controlOutputRaw.write(y);

        writeControlOutput(y);
    }
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
