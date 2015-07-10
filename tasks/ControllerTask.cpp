/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ControllerTask.hpp"

using namespace ctrl_lib;

ControllerTask::ControllerTask(std::string const& name)
    : ControllerTaskBase(name)
{
}

ControllerTask::ControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ControllerTaskBase(name, engine)
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
    controller->yMax = _maxControlOutput.get();
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

    _newPropGain.read((base::VectorXd&)controller->kp);
    _newMaxControlOutput.read((base::VectorXd&)controller->yMax);
    _newDeadZone.read((base::VectorXd&)controller->eMin);

    if(!readSetpoints() && state() != NO_SETPOINT)
        state(NO_SETPOINT);
    else if(!readFeedback() && state() != NO_FEEDBACK)
        state(NO_FEEDBACK);
    else{
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
