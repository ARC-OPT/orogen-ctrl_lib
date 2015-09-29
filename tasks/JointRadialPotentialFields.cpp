/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointRadialPotentialFields.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>

using namespace ctrl_lib;

JointRadialPotentialFields::JointRadialPotentialFields(std::string const& name)
    : JointRadialPotentialFieldsBase(name){
}

JointRadialPotentialFields::JointRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : JointRadialPotentialFieldsBase(name, engine){
}

JointRadialPotentialFields::~JointRadialPotentialFields(){
}

bool JointRadialPotentialFields::configureHook(){

    if (! JointRadialPotentialFieldsBase::configureHook())
        return false;

    field_names = _field_names.get();

    // Create a single 1-dimensional potential field per joint:
    std::vector<PotentialField*> fields;
    for(size_t i = 0; i < field_names.size(); i++)
        fields.push_back(new RadialPotentialField(1, _order.get()));

    // The Controller contains all potential fields:
    controller->setFields(fields);

    setInfluenceDistance(_initial_influence_distance.get());
    setPotFieldCenters(_initial_pot_field_centers.get());

    control_output.resize(field_names.size());
    control_output.names = field_names;

    return true;
}

bool JointRadialPotentialFields::startHook(){
    if (! JointRadialPotentialFieldsBase::startHook())
        return false;
    return true;
}

void JointRadialPotentialFields::updateHook(){
    JointRadialPotentialFieldsBase::updateHook();
}

void JointRadialPotentialFields::errorHook(){
    JointRadialPotentialFieldsBase::errorHook();
}

void JointRadialPotentialFields::stopHook(){
    JointRadialPotentialFieldsBase::stopHook();
}

void JointRadialPotentialFields::cleanupHook(){

    for(size_t i = 0; i < controller->fields.size(); i++)
        delete controller->fields[i];

    JointRadialPotentialFieldsBase::cleanupHook();
}

bool JointRadialPotentialFields::readPotFieldCenters(){

    if(_pot_field_centers.readNewest(pot_field_centers) == RTT::NewData)
        setPotFieldCenters(pot_field_centers);

    return true; //Always return true here, since we don't necessarily need a setpoint (could be the one from configuration)
}

bool JointRadialPotentialFields::readActualPosition(){

    if(_actual_position.read(actual_position) == RTT::NoData)
        return false;
    else{
        extractPositions(actual_position, field_names, position);
        for(size_t i = 0; i < field_names.size(); i++){
            controller->fields[i]->position.resize(1);
            controller->fields[i]->position[0] = position(i);
        }
        return true;
    }
}

void JointRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = ctrl_output_raw(i);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void JointRadialPotentialFields::setPotFieldCenters(const base::commands::Joints &centers){

    assert(controller != 0);

    extractPositions(centers, field_names, position);
    for(size_t i = 0; i < controller->fields.size(); i++){
        controller->fields[i]->pot_field_center.resize(1);
        controller->fields[i]->pot_field_center(0) = position(i);
    }
}
