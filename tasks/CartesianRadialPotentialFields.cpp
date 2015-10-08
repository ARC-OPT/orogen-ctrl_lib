/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>
#include <base/Logging.hpp>

using namespace ctrl_lib;

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name)
    : CartesianRadialPotentialFieldsBase(name){
}

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianRadialPotentialFieldsBase(name, engine){
}

CartesianRadialPotentialFields::~CartesianRadialPotentialFields(){
}

bool CartesianRadialPotentialFields::configureHook(){

    if (! CartesianRadialPotentialFieldsBase::configureHook())
        return false;

    field_names = _field_names.get();
    if(field_names.size() != 3){
        LOG_ERROR("%s: Size of field name vector has to be be 3, but is %i", field_names.size(), this->getName().c_str());
        return false;
    }

    influence_distance = _initial_influence_distance.get();
    order = _order.get();

    return true;
}

bool CartesianRadialPotentialFields::startHook(){
    if (! CartesianRadialPotentialFieldsBase::startHook())
        return false;
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
    clearPotentialFields();
    CartesianRadialPotentialFieldsBase::cleanupHook();
}

bool CartesianRadialPotentialFields::readPotFieldCenters(){
    _influence_distance.read(influence_distance);

    if(_pot_field_centers.readNewest(pot_field_centers) == RTT::NewData)
        has_pot_field_centers = true;

    if(has_pot_field_centers){
        setPotentialFieldCenters(pot_field_centers);
        return true;
    }
    else
        return false;

    return true;
}

bool CartesianRadialPotentialFields::readActualPosition(){
    if(_feedback.read(position) == RTT::NewData)
        has_position = true;
    if(has_position){
        setActualPosition(position);
        _current_feedback.write(position);
        return true;
    }
    else
        return false;
}

void CartesianRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    //Control output will only have linear components, since radial fields cannot generate rotational components
    for(uint i = 0; i < 3; i++)
        control_output.velocity(i) = ctrl_output_raw(i);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianRadialPotentialFields::setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers){

    assert(controller != 0);

    if(!centers.empty())
    {
        if(centers.size() != controller->fields.size()){

            clearPotentialFields();

            std::vector<PotentialField*> fields(centers.size());
            for(size_t i = 0; i < centers.size(); i++){
                fields[i] = new RadialPotentialField(3, order);
                fields[i]->influence_distance = influence_distance;
            }
            controller->setFields(fields);
        }
        for(size_t i = 0; i < controller->fields.size(); i++){
            if(!centers[i].hasValidPosition()){
                LOG_ERROR("%s: Potential fields center number %i (source: %s, target: %s) has invalid position, e.g. NaN",
                          this->getName().c_str(), i, centers[i].sourceFrame.c_str(), centers[i].targetFrame.c_str());
                throw std::invalid_argument("Invalid potential field centers");
            }
            controller->fields[i]->pot_field_center = centers[i].position;
            controller->fields[i]->position = position.position;
        }
    }
}

void CartesianRadialPotentialFields::setActualPosition(const base::samples::RigidBodyState& actual){

    for(size_t i = 0; i < controller->fields.size(); i++){
        if(!actual.hasValidPosition()){
            LOG_ERROR("%s: Actual position is invalid, e.g. NaN", this->getName().c_str());
            throw std::invalid_argument("Invalid actual position");
        }
        controller->fields[i]->position = position.position;
    }
}

void CartesianRadialPotentialFields::clearPotentialFields(){
    for(size_t i = 0; i < controller->fields.size(); i++)
        delete controller->fields[i];
    controller->fields.clear();
    pot_field_centers.clear();
    position.invalidatePosition();
}
