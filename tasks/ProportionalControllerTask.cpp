/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ProportionalControllerTask.hpp"
#include <base/Logging.hpp>

using namespace ctrl_lib;

ProportionalControllerTask::ProportionalControllerTask(std::string const& name)
    : ProportionalControllerTaskBase(name)
{
}

ProportionalControllerTask::ProportionalControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ProportionalControllerTaskBase(name, engine)
{
}

ProportionalControllerTask::~ProportionalControllerTask()
{
}
bool ProportionalControllerTask::configureHook()
{
    if (! ProportionalControllerTaskBase::configureHook())
        return false;

    field_names = _field_names.get();
    controller = new ProportionalController(field_names.size());

    controller->prop_gain = _initial_prop_gain.get();
    if((size_t)controller->prop_gain.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial proportional gain has size %i",
                  this->getName().c_str(), field_names.size(), controller->prop_gain.size());
        return false;
    }

    controller->max_control_output = _initial_max_control_output.get();
    if((size_t)controller->max_control_output.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial max control output has size %i",
                  this->getName().c_str(), field_names.size(), controller->max_control_output.size());
        return false;
    }

    controller->dead_zone = _initial_dead_zone.get();
    if((size_t)controller->dead_zone.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial dead zone has size %i",
                  this->getName().c_str(), field_names.size(), controller->dead_zone.size());
        return false;
    }

    return true;
}
bool ProportionalControllerTask::startHook()
{
    if (! ProportionalControllerTaskBase::startHook())
        return false;

    has_setpoint = has_feedback = false;

    return true;
}
void ProportionalControllerTask::updateHook()
{
    ProportionalControllerTaskBase::updateHook();

    _prop_gain.readNewest((base::VectorXd&)controller->prop_gain);
    _max_control_output.readNewest((base::VectorXd&)controller->max_control_output);
    _dead_zone.readNewest((base::VectorXd&)controller->dead_zone);

    _current_prop_gain.write(controller->prop_gain);
    _current_max_control_output.write(controller->max_control_output);
    _current_dead_zone.write(controller->dead_zone);

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

    _control_error.write(controller->control_error);

    writeControlOutput(control_output_raw);

}
void ProportionalControllerTask::errorHook()
{
    ProportionalControllerTaskBase::errorHook();
}
void ProportionalControllerTask::stopHook()
{
    ProportionalControllerTaskBase::stopHook();
}
void ProportionalControllerTask::cleanupHook()
{
    ProportionalControllerTaskBase::cleanupHook();
    delete controller;
}