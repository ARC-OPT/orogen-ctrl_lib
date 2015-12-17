/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ProportionalControllerTask.hpp"
#include <base/Logging.hpp>
#include <ctrl_lib/ProportionalController.hpp>

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

    ProportionalController* ctrl = new ProportionalController(field_names.size());

    ctrl->max_control_output = _max_control_output.get();
    if((size_t)ctrl->max_control_output.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial max control output has size %i",
                  this->getName().c_str(), field_names.size(), ctrl->max_control_output.size());
        return false;
    }

    ctrl->dead_zone = _dead_zone.get();
    if((size_t)ctrl->dead_zone.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial dead zone has size %i",
                  this->getName().c_str(), field_names.size(), ctrl->dead_zone.size());
        return false;
    }

    ctrl->prop_gain = _prop_gain.get();
    if((size_t)ctrl->prop_gain.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial proportional gain has size %i",
                  this->getName().c_str(), field_names.size(), ctrl->prop_gain.size());
        return false;
    }
    ctrl->ff_gain = _ff_gain.get();
    if((size_t)ctrl->ff_gain.size() != field_names.size()){
        LOG_ERROR("%s: Field names has size %i, but initial feed forward gain has size %i",
                  this->getName().c_str(), field_names.size(), ctrl->ff_gain.size());
        return false;
    }

    controller = ctrl;

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
    ProportionalController* ctrl = (ProportionalController*)controller;

    ctrl->max_control_output = _max_control_output.get();
    ctrl->dead_zone = _dead_zone.get();
    ctrl->prop_gain = _prop_gain.get();
    ctrl->ff_gain = _ff_gain.get();

    _current_max_control_output.write(ctrl->max_control_output);
    _current_dead_zone.write(ctrl->dead_zone);
    _current_prop_gain.write(ctrl->prop_gain);
    _current_ff_gain.write(ctrl->ff_gain);

    ProportionalControllerTaskBase::updateHook();
}

void ProportionalControllerTask::writeActivationFunction()
{
    ProportionalController* ctrl = (ProportionalController*)controller;
    activation.resize(field_names.size());

    for(int i = 0; i < control_output_raw.size(); i++)
        activation(i) = fabs(control_output_raw(i))/ctrl->max_control_output(i);

    _activation.write(activation_function.compute(activation));
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
