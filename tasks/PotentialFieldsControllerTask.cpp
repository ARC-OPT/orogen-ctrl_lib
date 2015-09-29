/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PotentialFieldsControllerTask.hpp"
#include <ctrl_lib/PotentialFieldsController.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

PotentialFieldsControllerTask::PotentialFieldsControllerTask(std::string const& name, TaskCore::TaskState initial_state)
    : PotentialFieldsControllerTaskBase(name, initial_state)
{
}

PotentialFieldsControllerTask::PotentialFieldsControllerTask(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : PotentialFieldsControllerTaskBase(name, engine, initial_state)
{
}

PotentialFieldsControllerTask::~PotentialFieldsControllerTask()
{
}

bool PotentialFieldsControllerTask::configureHook()
{
    if (! PotentialFieldsControllerTaskBase::configureHook())
        return false;

    controller = new PotentialFieldsController();

    field_names = _field_names.get();
    if(field_names.empty()){
        LOG_ERROR("%s: Size of field_names must be > 0", this->getName().c_str());
        return false;
    }

    controller->prop_gain = _initial_prop_gain.get();
    if(field_names.size() != (size_t)controller->prop_gain.size()){
        LOG_ERROR("%s: Size of field_names is %i, but size of proportional gain is %i",
                  this->getName().c_str(), field_names.size(), controller->prop_gain.size());
        return false;
    }
    controller->max_control_output = _initial_max_control_output.get();
    if(field_names.size() != (size_t)controller->max_control_output.size()){
        LOG_ERROR("%s: Size of field_names is %i, but size of maximum control output is %i",
                  this->getName().c_str(), field_names.size(), controller->max_control_output.size());
        return false;
    }

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
    PotentialFieldsControllerTaskBase::updateHook();

    _prop_gain.readNewest((base::VectorXd&)controller->prop_gain);
    _max_control_output.readNewest((base::VectorXd&)controller->max_control_output);

    _current_prop_gain.write(controller->prop_gain);
    _current_max_control_output.write(controller->max_control_output);

    if(!readActualPosition()){
        if(state() != NO_ACTUAL_POSITION)
            state(NO_ACTUAL_POSITION);
        return;
    }
    if(!readPotFieldCenters()){
        if(state() != NO_POT_FIELD_CENTERS)
            state(NO_POT_FIELD_CENTERS);
        return;
    }

    if(state() != RUNNING)
        state(RUNNING);

    controller->update(control_output_raw);
    writeControlOutput(control_output_raw);

    field_infos.resize(controller->fields.size());
    for(size_t i = 0; i < controller->fields.size(); i++)
        field_infos[i].setFromField(controller->fields[i]);
    _field_infos.write(field_infos);

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
}
