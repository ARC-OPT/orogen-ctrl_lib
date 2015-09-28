/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"
#include <ctrl_lib/MultiPotentialFields3D.hpp>
#include <ctrl_lib/RadialRepulsivePotentialField.hpp>
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

    field_names = _field_names.get();
    if(field_names.size() != 3){
        LOG_ERROR("Size of field name vector should be 3, but is %i", field_names.size());
        return false;
    }

    pot_field_centers = _pot_field_centers.get();
    influence_distance = _initial_influence_distance.get();
    order = _order.get();

    //Create potential fields. All fields will have dimension 3 here!
    std::vector<PotentialField*> multiFields;
    for(uint i = 0; i < pot_field_centers.size(); i++)
        multiFields.push_back(new RadialRepulsivePotentialField(3));

    // The Controller contains all potential fields:
    controller = new MultiPotentialFields3D(multiFields);

    setMaxInfluenceDistance(influence_distance);
    setOrder(order);
    setPotentialFieldCenters(pot_field_centers);

    if (! CartesianRadialPotentialFieldsBase::configureHook())
        return false;

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
    MultiPotentialFields3D* multi_field_ctrl = (MultiPotentialFields3D*)controller;
    for(size_t i = 0; i < multi_field_ctrl->fields.size(); i++)
        delete multi_field_ctrl->fields[i];
    delete controller;
    controller = 0;
    CartesianRadialPotentialFieldsBase::cleanupHook();
}

bool CartesianRadialPotentialFields::readSetpoints(){
    if(_setpoint.readNewest(pot_field_centers) == RTT::NewData)
        setPotentialFieldCenters(pot_field_centers);

    if(_influence_distance.read(influence_distance) == RTT::NewData)
        setMaxInfluenceDistance(influence_distance);

    return true; //Always return true here, since we don't necessarily need a setpoint (could be fixed by configuration)
}

bool CartesianRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        MultiPotentialFields3D* multi_field_ctrl = (MultiPotentialFields3D*)controller;
        for(size_t i = 0; i < multi_field_ctrl->fields.size(); i++)
            multi_field_ctrl->fields[i]->position = feedback.position;
        return true;
    }
}

void CartesianRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    //Control output will only have linear components, since radial fields cannot generate rotational components
    for(uint i = 0; i < 3; i++)
        control_output.velocity(i) = ctrl_output_raw(i);

    //Write gradients of all potential fields on debug port
    MultiPotentialFields3D* multi_field_ctrl = (MultiPotentialFields3D*)controller;
    gradients.resize(multi_field_ctrl->fields.size());
    for(size_t i = 0; i < multi_field_ctrl->fields.size(); i++)
        gradients[i] = multi_field_ctrl->gradients[i];
    _gradients.write(gradients);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void CartesianRadialPotentialFields::setMaxInfluenceDistance(const base::VectorXd& distance){

    assert(controller != 0);

    // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
    // only do sth. if the size is not zero
    if(distance.size() == 0){

        MultiPotentialFields3D* multi_field_ctrl = (MultiPotentialFields3D*)controller;
        if(distance.size() != (int)multi_field_ctrl->fields.size()){
            LOG_ERROR("maxInfluenceDistance vector must have same length as number of potential fields");
            throw std::invalid_argument("Invalid input");
        }
        for(size_t i = 0; i < multi_field_ctrl->fields.size(); i++)
            multi_field_ctrl->fields[i]->influence_distance = distance(i);
    }
}

void CartesianRadialPotentialFields::setOrder(const double order){

    assert(controller != 0);
    MultiPotentialFields3D* multi_field_ctrl = (MultiPotentialFields3D*)controller;
    for(size_t i = 0; i < multi_field_ctrl->fields.size(); i++)
        multi_field_ctrl->fields[i]->order = order;
}

void CartesianRadialPotentialFields::setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers){

    assert(controller != 0);

    MultiPotentialFields3D* multi_field_ctrl = (MultiPotentialFields3D*)controller;
    if(centers.size() != multi_field_ctrl->fields.size()){
        LOG_ERROR("Pot. Field Center vector must have same length as number of potential fields");
        throw std::invalid_argument("Invalid input");
    }
    for(size_t i = 0; i < multi_field_ctrl->fields.size(); i++)
        multi_field_ctrl->fields[i]->pot_field_center = centers[i].position;
}

