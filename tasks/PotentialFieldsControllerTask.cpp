/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PotentialFieldsControllerTask.hpp"
#include <ctrl_lib/PotentialFieldsController.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

PotentialFieldsControllerTask::PotentialFieldsControllerTask(std::string const& name)
    : PotentialFieldsControllerTaskBase(name)
{
}

PotentialFieldsControllerTask::PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PotentialFieldsControllerTaskBase(name, engine)
{
}

PotentialFieldsControllerTask::~PotentialFieldsControllerTask()
{
}

bool PotentialFieldsControllerTask::configureHook()
{
    if (! PotentialFieldsControllerTaskBase::configureHook())
        return false;

    has_pot_field_centers = has_feedback = false;

    field_names = _field_names.get();
    PotentialFieldsController* ctrl = new PotentialFieldsController(field_names.size());

    if(field_names.empty()){
        LOG_ERROR("%s: Size of field_names must be > 0", this->getName().c_str());
        return false;
    }

    ctrl->prop_gain = _prop_gain.get();
    if(field_names.size() != (size_t)ctrl->prop_gain.size()){
        LOG_ERROR("%s: Size of field_names is %i, but size of proportional gain is %i",
                  this->getName().c_str(), field_names.size(), ctrl->prop_gain.size());
        return false;
    }
    ctrl->max_control_output = _max_control_output.get();
    if(field_names.size() != (size_t)ctrl->max_control_output.size()){
        LOG_ERROR("%s: Size of field_names is %i, but size of maximum control output is %i",
                  this->getName().c_str(), field_names.size(), ctrl->max_control_output.size());
        return false;
    }

    influence_distance = _influence_distance.get();

    controller = ctrl;

    return true;
}
bool PotentialFieldsControllerTask::startHook()
{
    if (! PotentialFieldsControllerTaskBase::startHook())
        return false;

    return true;
}
void PotentialFieldsControllerTask::updateHook()
{
    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

    ctrl->prop_gain = _prop_gain.get();
    ctrl->max_control_output = _max_control_output.get();
    influence_distance = _influence_distance.get();
    setInfluenceDistance(influence_distance);

    _current_prop_gain.write(ctrl->prop_gain);
    _current_max_control_output.write(ctrl->max_control_output);
    _current_influence_distance.write(influence_distance);

    field_infos.resize(ctrl->fields.size());
    for(size_t i = 0; i < ctrl->fields.size(); i++)
        field_infos[i].setFromField(ctrl->fields[i]);
    _field_infos.write(field_infos);

    PotentialFieldsControllerTaskBase::updateHook();

}
void PotentialFieldsControllerTask::errorHook()
{
    PotentialFieldsControllerTaskBase::errorHook();
}
void PotentialFieldsControllerTask::stopHook()
{
    PotentialFieldsControllerTaskBase::stopHook();
}
void PotentialFieldsControllerTask::cleanupHook()
{
    PotentialFieldsControllerTaskBase::cleanupHook();

    delete controller;
    field_names.clear();
    field_infos.clear();
}
