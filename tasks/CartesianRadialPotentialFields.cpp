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

    controller = new PotentialFieldsController(3);

    if (! CartesianRadialPotentialFieldsBase::configureHook())
        return false;

    field_names = _field_names.get();
    if(field_names.size() != 3){
        LOG_ERROR("%s: Size of field name vector has to be be 3, but is %i", field_names.size(), this->getName().c_str());
        return false;
    }
    order = _order.get();

    return true;
}
bool CartesianRadialPotentialFields::readSetpoint(){

    if(_pot_field_centers.readNewest(pot_field_centers) == RTT::NewData)
        setPotentialFieldCenters(pot_field_centers);

    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;
    return ctrl->hasPotFieldCenters();
}

bool CartesianRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NewData){
        if(!feedback.hasValidPosition()){
            LOG_ERROR("%s: Actual position is invalid, e.g. NaN", this->getName().c_str());
            throw std::invalid_argument("Invalid actual position");
        }
        controller->setFeedback(feedback.position);
    }
    return controller->hasFeedback();
}

void CartesianRadialPotentialFields::writeControlOutput(const base::VectorXd &ctrl_output_raw){
    //Control output will only have linear components, since radial fields cannot generate rotational components
    for(uint i = 0; i < 3; i++)
        control_output.velocity(i) = ctrl_output_raw(i);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianRadialPotentialFields::setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers){

    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

    assert(ctrl != 0);

    if(!centers.empty())
    {
        if(centers.size() != ctrl->getNoOfFields()){

            ctrl->clearPotentialFields();

            std::vector<PotentialField*> fields(centers.size());
            for(size_t i = 0; i < centers.size(); i++)
            {
                if(!centers[i].hasValidPosition()){
                    LOG_ERROR("%s: Potential fields center number %i (source: %s, target: %s) has invalid position, e.g. NaN",
                              this->getName().c_str(), i, centers[i].sourceFrame.c_str(), centers[i].targetFrame.c_str());
                    throw std::invalid_argument("Invalid potential field centers");
                }

                fields[i] = new RadialPotentialField(3, order);
                fields[i]->pot_field_center = centers[i].position;
            }
            ctrl->setFields(fields);
            ctrl->setInfluenceDistance(_influence_distance.get());
        }
    }
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
    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;
    ctrl->clearPotentialFields();
    CartesianRadialPotentialFieldsBase::cleanupHook();
}
