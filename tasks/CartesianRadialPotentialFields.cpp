/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"
#include <wbc/controllers/RadialPotentialField.hpp>
#include <base-logging/Logging.hpp>
#include <wbc/controllers/CartesianPotentialFieldsController.hpp>

using namespace ctrl_lib;

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name)
    : CartesianRadialPotentialFieldsBase(name){
}

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianRadialPotentialFieldsBase(name, engine){
}

bool CartesianRadialPotentialFields::configureHook(){
    if(!CartesianRadialPotentialFieldsBase::configureHook())
        return false;

    controller = new CartesianPotentialFieldsController();
    controller->setPGain(_p_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());
    influence_distance = _influence_distance.get();

    return true;
}

bool CartesianRadialPotentialFields::startHook(){
    if(!CartesianRadialPotentialFieldsBase::startHook())
        return false;

    controller->clearFields();
    _pot_field_centers.clear();
    _feedback.clear();

    return true;
}

void CartesianRadialPotentialFields::updateHook(){
    CartesianRadialPotentialFieldsBase::updateHook();
}

void CartesianRadialPotentialFields::errorHook(){
    CartesianRadialPotentialFieldsBase::errorHook();
}

void CartesianRadialPotentialFields::stopHook(){
    CartesianRadialPotentialFieldsBase::stopHook();
}


void CartesianRadialPotentialFields::cleanupHook(){
    CartesianRadialPotentialFieldsBase::cleanupHook();
    delete controller;
}

bool CartesianRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        return true;
    }
}

bool CartesianRadialPotentialFields::readSetpoint(){
    if(_pot_field_centers.readNewest(pot_field_centers) == RTT::NoData)
        return false;
    else
        return true;
}

void CartesianRadialPotentialFields::updateController(){

    std::vector<PotentialFieldPtr> fields;
    for(size_t i = 0; i < pot_field_centers.size(); i++)
    {
        // Only use the fields with correct frame ID
        if(pot_field_centers[i].targetFrame == feedback.source_frame){

            PotentialFieldPtr field = std::make_shared<RadialPotentialField>(3, pot_field_centers[i].sourceFrame);
            field->pot_field_center = pot_field_centers[i].position;
            field->influence_distance = influence_distance;
            fields.push_back(field);
        }
    }
    controller->setFields(fields);
    control_output = ((CartesianPotentialFieldsController*)controller)->update(feedback);

    _control_output.write(control_output);
    _field_infos.write(controller->getFieldInfos());
}

const base::VectorXd& CartesianRadialPotentialFields::computeActivation(ActivationFunction &activation_function){
    tmp.resize(6);
    tmp.setZero();

    // Get highest activation from all fields
    double max_activation = 0;
    const std::vector<PotentialFieldPtr> &fields = controller->getFields();
    for(uint i = 0; i < fields.size(); i++){
        double activation;
        if(fields[i]->distance.norm() > fields[i]->influence_distance)
            activation = 0;
        else
            activation = fabs(fields[i]->distance.norm() - fields[i]->influence_distance) / fields[i]->influence_distance;
        if(activation > max_activation)
            max_activation = activation;
    }

    // Use maximum activation for all dof
    for(uint i = 0; i < controller->getDimension(); i++)
        tmp(i) = max_activation;

    return activation_function.compute(tmp);
}
