/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointRadialPotentialFields.hpp"
#include <ctrl_lib/RadialRepulsivePotentialField.hpp>
#include <ctrl_lib/MultiPotentialFields1D.hpp>

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

    field_names = _field_names.get();
    influence_distance = _initial_influence_distance.get();
    pot_field_centers = _pot_field_centers.get();
    order = _order.get();

    // Create a single 1-dimensional potential field per joint:
    std::vector<PotentialField*> multiFields;
    for(size_t i = 0; i < field_names.size(); i++)
        multiFields.push_back(new RadialRepulsivePotentialField(1));

    // The Controller contains all potential fields:
    controller = new MultiPotentialFields1D(multiFields, field_names.size());

    setMaxInfluenceDistance(influence_distance);
    setPotFieldCenters(pot_field_centers);
    setOrder(order);

    control_output.resize(field_names.size());
    control_output.names = field_names;

    if (! JointRadialPotentialFieldsBase::configureHook())
        return false;

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

    MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        delete multiFieldCtrl->fields[i];
    delete controller;
    controller = 0;

    JointRadialPotentialFieldsBase::cleanupHook();
}

bool JointRadialPotentialFields::readSetpoints(){

    if(_setpoint.readNewest(pot_field_centers) == RTT::NewData)
        setPotFieldCenters(pot_field_centers);

    if(_influence_distance.read(influence_distance) == RTT::NewData)
        setMaxInfluenceDistance(influence_distance);

    return true; //Always return true here, since we don't necessarily need a setpoint (could be fixed by configuration)
}

bool JointRadialPotentialFields::readFeedback(){

    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
        base::JointState status;
        multiFieldCtrl->feedback.resize(field_names.size());
        for(size_t i = 0; i < field_names.size(); i++){
            try{
                status = feedback.getElementByName(field_names[i]);
                if(!status.hasPosition()){ //Throw here, since we always need valid positions for this controller
                    LOG_ERROR("Element %s has no valid position value!", field_names[i].c_str());
                    throw std::invalid_argument("Invalid position value");
                }
            }
            catch(std::exception e){
                LOG_ERROR("Feedback vector does not contain element %s", field_names[i].c_str());
                throw e;
            }

            multiFieldCtrl->feedback(i) = status.position;
        }
        return true;
    }
}

void JointRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &ctrl_output_raw){
    for(size_t i = 0; i < field_names.size(); i++){
        control_output[i].speed = ctrl_output_raw(i);
    }
    //Write gradients of all potential fields on debug port
    MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
    gradients.resize(multiFieldCtrl->fields.size());
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        gradients[i] = multiFieldCtrl->gradients[i];
    _gradients.write(gradients);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void JointRadialPotentialFields::setMaxInfluenceDistance(const base::VectorXd &distance){

    assert(controller != 0);

    // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
    // only do sth. if the size is not zero
    if(distance.size() != 0){

        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
        if(distance.size() != (int)multiFieldCtrl->fields.size()){
            LOG_ERROR("Max. Influence Distance vector must have same length as number of potential fields");
            throw std::invalid_argument("Invalid input");
        }
        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->influence_distance = distance(i);
    }
}

void JointRadialPotentialFields::setPotFieldCenters(const base::commands::Joints &centers){

    assert(controller != 0);

    MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
    if(centers.size() != multiFieldCtrl->fields.size()){
        LOG_ERROR("Pot. Field Center vector must have same length as number of potential fields");
        throw std::invalid_argument("Invalid input");
    }
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        multiFieldCtrl->setpoint[i] = centers[i].position;
}

void JointRadialPotentialFields::setOrder(const double order){

    assert(controller != 0);
    MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        multiFieldCtrl->fields[i]->order = order;
}
