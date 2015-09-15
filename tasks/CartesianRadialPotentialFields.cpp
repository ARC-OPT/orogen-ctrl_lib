/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"
#include <ctrl_lib/MultiPotentialFields.hpp>
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

    std::vector<std::string> fieldNames = _fieldNames.get();
    potFieldCenters = _potFieldCenters.get();
    maxInfluenceDistance = _maxInfluenceDistance.get();
    potFieldOrder = _order.get();

    if(fieldNames.size() != 3){
        LOG_ERROR("Size of field name vector should be 3, but is %i", fieldNames.size());
        return false;
    }

    //Create potential fields. All fields will have dimension 3 here!
    std::vector<PotentialField*> multiFields;
    for(uint i = 0; i < potFieldCenters.size(); i++)
        multiFields.push_back(new RadialRepulsivePotentialField(3));

    // The Controller contains all potential fields:
    controller = new MultiPotentialFields(multiFields, 3);

    setMaxInfluenceDistance(maxInfluenceDistance);
    setOrder(potFieldOrder);
    setPotentialFieldCenters(potFieldCenters);

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
    MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        delete multiFieldCtrl->fields[i];
    delete controller;
    controller = 0;
    CartesianRadialPotentialFieldsBase::cleanupHook();
}

bool CartesianRadialPotentialFields::readSetpoints(){
    if(_setpoint.readNewest(potFieldCenters) == RTT::NewData)
        setPotentialFieldCenters(potFieldCenters);

    if(_newMaxInfluenceDistance.read(maxInfluenceDistance) == RTT::NewData)
        setMaxInfluenceDistance(maxInfluenceDistance);

    if(_newOrder.read(potFieldOrder) == RTT::NewData)
        setOrder(potFieldOrder);

    return true; //Always return true here, since we don't necessarily need a setpoint (could be fixed by configuration)
}

bool CartesianRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->x = feedback.position;

        _currentFeedback.write(feedback);
        return true;
    }
}

void CartesianRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &y){
    //Control output will only have linear components, since radial fields cannot generate rotational components
    for(uint i = 0; i < 3; i++)
        controlOutput.velocity(i) = y(i);

    //Write gradients of all potential fields on debug port
    MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
    gradients.resize(multiFieldCtrl->fields.size());
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        gradients[i] = multiFieldCtrl->gradients[i];
    _gradients.write(gradients);

    controlOutput.time = base::Time::now();
    _controlOutput.write(controlOutput);
}

void CartesianRadialPotentialFields::setMaxInfluenceDistance(const base::VectorXd& distance){

    assert(controller != 0);

    // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
    // only do sth. if the size is not zero
    if(distance.size() == 0){

        MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
        if(distance.size() != (int)multiFieldCtrl->fields.size()){
            LOG_ERROR("maxInfluenceDistance vector must have same length as number of potential fields");
            throw std::invalid_argument("Invalid input");
        }
        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->dMax = distance(i);
    }
}

void CartesianRadialPotentialFields::setOrder(const double order){

    assert(controller != 0);
    MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        multiFieldCtrl->fields[i]->M = order;
}

void CartesianRadialPotentialFields::setPotentialFieldCenters(const std::vector<base::samples::RigidBodyState> &centers){

    assert(controller != 0);

    MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
    if(centers.size() != multiFieldCtrl->fields.size()){
        LOG_ERROR("Pot. Field Center vector must have same length as number of potential fields");
        throw std::invalid_argument("Invalid input");
    }
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        multiFieldCtrl->fields[i]->x0 = centers[i].position;
}

