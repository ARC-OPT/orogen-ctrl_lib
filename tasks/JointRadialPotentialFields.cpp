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
    pot_field_centers = _initial_pot_field_centers.get();
    if(!pot_field_centers.empty()){
        setPotFieldCenters(pot_field_centers);
        has_pot_field_centers = true;
    }

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

    pot_field_centers.clear();
    position.clear();
    control_output.clear();

    JointRadialPotentialFieldsBase::cleanupHook();
}

bool JointRadialPotentialFields::readPotFieldCenters(){

    if(_pot_field_centers.readNewest(pot_field_centers) == RTT::NewData)
        has_pot_field_centers = true;

    if(has_pot_field_centers){
        setPotFieldCenters(pot_field_centers);
        return true;
    }
    else
        return false;

    return true;
}

bool JointRadialPotentialFields::readActualPosition(){

    if(_feedback.readNewest(position) == RTT::NewData)
        has_position = true;

    if(has_position){
        extractPositions(position, field_names, position_raw);
        for(size_t i = 0; i < field_names.size(); i++){
            controller->fields[i]->position.resize(1);
            controller->fields[i]->position[0] = position_raw(i);
        }
        _current_position.write(position);
        return true;
    }
    else
        return false;
}

void JointRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = ctrl_output_raw(i);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void JointRadialPotentialFields::setPotFieldCenters(const base::commands::Joints &centers){

    assert(controller != 0);

    extractPositions(centers, field_names, position_raw);
    for(size_t i = 0; i < controller->fields.size(); i++){
        controller->fields[i]->pot_field_center.resize(1);
        controller->fields[i]->pot_field_center(0) = position_raw(i);
    }
}
