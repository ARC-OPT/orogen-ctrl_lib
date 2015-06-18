/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PositionControllerBase.hpp"
#include <ctrl_lib/PositionControlFeedForward.hpp>

using namespace ctrl_lib;

PositionControllerBase::PositionControllerBase(std::string const& name)
    : PositionControllerBaseBase(name)
{
}

PositionControllerBase::PositionControllerBase(std::string const& name, RTT::ExecutionEngine* engine)
    : PositionControllerBaseBase(name, engine)
{
}

PositionControllerBase::~PositionControllerBase()
{
}

bool PositionControllerBase::configureHook()
{
    if (! PositionControllerBaseBase::configureHook())
        return false;

    controller->kp = _kp.get();
    controller->kd = _kd.get();
    controller->yMax = _maxCtrlOutput.get();
    controller->eMin = _deadZone.get();

    return true;
}
bool PositionControllerBase::startHook()
{
    if (! PositionControllerBaseBase::startHook())
        return false;
    return true;
}
void PositionControllerBase::updateHook()
{
    PositionControllerBaseBase::updateHook();

    _kpNew.read((base::VectorXd&)controller->kp);
    _kdNew.read((base::VectorXd&)controller->kd);
    _maxCtrlOutputNew.read((base::VectorXd&)controller->yMax);
    _deadZoneNew.read((base::VectorXd&)controller->eMin);

    if(!readSetpoints() && state() != NO_SETPOINT)
        state(NO_SETPOINT);
    else if(!readFeedback() && state() != NO_FEEDBACK)
        state(NO_FEEDBACK);
    else{
        controller->update(x,xr,v,vr,ar,y);
        sendControlOutput();
    }

}
void PositionControllerBase::errorHook()
{
    PositionControllerBaseBase::errorHook();
}
void PositionControllerBase::stopHook()
{
    PositionControllerBaseBase::stopHook();
}
void PositionControllerBase::cleanupHook()
{
    PositionControllerBaseBase::cleanupHook();
}
