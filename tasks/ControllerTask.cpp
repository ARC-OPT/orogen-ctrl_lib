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
    controller->prop_gain = _initial_prop_gain.get();
    if((size_t)controller->prop_gain.size() != field_names.size()){
        LOG_ERROR("%s: Initial proportional Gain should have size %i, but has size %i",
                  this->getName().c_str(), field_names.size(), controller->prop_gain.size());
        return false;
    }

    controller->max_control_output = _initial_max_control_output.get();
    if((size_t)controller->max_control_output.size() != field_names.size()){
        LOG_ERROR("%s: Initial max control output should have size %i, but has size %i",
                  this->getName().c_str(), field_names.size(), controller->max_control_output.size());
        return false;
    }

    controller->dead_zone = _initial_dead_zone.get();
    if((size_t)controller->dead_zone.size() != field_names.size()){
        LOG_ERROR("%s: Initial dead zone should have size %i, but has size %i",
                  this->getName().c_str(), field_names.size(), controller->dead_zone.size());
        return false;
    }


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

    _prop_gain.readNewest((base::VectorXd&)controller->prop_gain);
    _max_control_output.readNewest((base::VectorXd&)controller->max_control_output);
    _dead_zone.readNewest((base::VectorXd&)controller->dead_zone);

    if(!readFeedback()){
        if(state() != NO_FEEDBACK)
            state(NO_FEEDBACK);
        return;
    }

    if(!readSetpoints()){
        if(state() != NO_SETPOINT)
            state(NO_SETPOINT);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    controller->update(control_output);
    writeControlOutput(control_output);
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
