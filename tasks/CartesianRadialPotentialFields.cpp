/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>
#include <base/Logging.hpp>
#include <ctrl_lib/CartesianPotentialFieldsController.hpp>

using namespace ctrl_lib;

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name)
    : CartesianRadialPotentialFieldsBase(name){
}

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianRadialPotentialFieldsBase(name, engine){
}

bool CartesianRadialPotentialFields::configureHook(){
    controller = new CartesianPotentialFieldsController(_field_names.get().size());
    if(!CartesianRadialPotentialFieldsBase::configureHook())
        return false;
    return true;
}

void CartesianRadialPotentialFields::cleanupHook(){
    CartesianRadialPotentialFieldsBase::cleanupHook();
    delete controller;
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
        controller->setPosition(feedback.position);
    }
    return controller->hasPosition();
}

void CartesianRadialPotentialFields::writeControlOutput(const base::VectorXd &ctrl_output_raw){
    //Control output will only have linear components, since radial fields cannot generate rotational components
    for(uint i = 0; i < 3; i++)
        control_output.velocity(i) = ctrl_output_raw(i);
    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianRadialPotentialFields::setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers){

    if(!centers.empty())
    {
        if(centers.size() != controller->getNoOfFields()){

            controller->clearPotentialFields();

            std::vector<PotentialField*> fields(centers.size());
            for(size_t i = 0; i < centers.size(); i++)
            {
                if(!centers[i].hasValidPosition()){
                    LOG_ERROR("%s: Potential fields center number %i (source: %s, target: %s) has invalid position, e.g. NaN",
                              this->getName().c_str(), i, centers[i].sourceFrame.c_str(), centers[i].targetFrame.c_str());
                    throw std::invalid_argument("Invalid potential field centers");
                }

                fields[i] = new RadialPotentialField(3);
                fields[i]->pot_field_center = centers[i].position;
            }
            controller->setFields(fields);
            controller->setInfluenceDistance(_influence_distance.get());
        }
    }
}

const base::VectorXd& CartesianRadialPotentialFields::computeActivation(ActivationFunction &activation_function){
    tmp.resize(6);
    tmp.setZero();

    // Get highest activation from all fields
    double max_activation = 0;
    const std::vector<PotentialField*> &fields = controller->getFields();
    for(uint i = 0; i < fields.size(); i++){
        double activation = fabs(fields[i]->distance.norm() - fields[i]->influence_distance) / fields[i]->influence_distance;
        if(activation > max_activation)
            max_activation = activation;
    }

    // Use maximum activation for all dof
    for(uint i = 0; i < controller->getDimension(); i++)
        tmp(i) = max_activation;

    return activation_function.compute(tmp);
}
