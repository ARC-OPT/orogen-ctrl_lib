/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ProportionalControllerTask.hpp"

using namespace ctrl_lib;

ProportionalControllerTask::ProportionalControllerTask(std::string const& name)
    : ProportionalControllerTaskBase(name){
    controller = 0;
}

ProportionalControllerTask::ProportionalControllerTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ProportionalControllerTaskBase(name, engine){
    controller = 0;
}

bool ProportionalControllerTask::configureHook(){
    controller = new ProportionalFeedForwardController(_field_names.get().size());
    if (! ProportionalControllerTaskBase::configureHook())
        return false;
    return true;
}

bool ProportionalControllerTask::startHook(){
    if (! ProportionalControllerTaskBase::startHook())
        return false;
    controller->clearSetpoint();
    controller->clearFeedback();
    return true;
}

void ProportionalControllerTask::cleanupHook(){
    ProportionalControllerTaskBase::cleanupHook();
    delete controller;
    controller = 0;
}

void ProportionalControllerTask::updateControllerProperties(){

    controller->setPropGain(_prop_gain.get());
    if(_ff_gain.get().size() > 0)
        controller->setFeedforwardGain(_ff_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());
    controller->setDeadZone(_dead_zone.get());

    _current_prop_gain.write(controller->getPropGain());
    if(_ff_gain.get().size() > 0)
        _current_ff_gain.write(controller->getFeedForwardGain());
    _current_max_control_output.write(controller->getMaxControlOutput());
    _current_dead_zone.write(controller->getDeadZone());
}

const base::VectorXd& ProportionalControllerTask::updateController(){
    const base::VectorXd &control_output = controller->update();
    _control_error.write(controller->getControlError());
    return control_output;
}

const base::VectorXd& ProportionalControllerTask::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    for(uint i = 0; i < controller->getDimension(); i++)
        tmp(i) = fabs(controller->getControlOutput()(i))/controller->getMaxControlOutput()(i);
    return activation_function.compute(tmp);
}
