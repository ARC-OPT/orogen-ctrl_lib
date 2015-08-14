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

    jointNames = _fieldNames.get();
    maxInfluenceDistance = _maxInfluenceDistance.get();
    potFieldCenters = _potFieldCenters.get();
    potFieldOrder = _order.get();

    // Create a single 1-dimensional potential field per joint:
    std::vector<PotentialField*> multiFields;
    for(size_t i = 0; i < jointNames.size(); i++)
        multiFields.push_back(new RadialRepulsivePotentialField(1));

    // The Controller contains all potential fields:
    controller = new MultiPotentialFields1D(multiFields, jointNames.size());

    setMaxInfluenceDistance(maxInfluenceDistance);
    setPotFieldCenters(potFieldCenters);
    setOrder(potFieldOrder);

    controlOutput.resize(jointNames.size());
    controlOutput.names = jointNames;

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

    if(_setpoint.readNewest(potFieldCenters) == RTT::NewData)
        setPotFieldCenters(potFieldCenters);

    if(_newMaxInfluenceDistance.read(maxInfluenceDistance) == RTT::NewData)
        setMaxInfluenceDistance(maxInfluenceDistance);

    if(_newOrder.read(potFieldOrder) == RTT::NewData)
        setOrder(potFieldOrder);

    return true; //Always return true here, since we don't necessarily need a setpoint (could be fixed by configuration)
}

bool JointRadialPotentialFields::readFeedback(){

    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
        base::JointState cmd;
        for(size_t i = 0; i < jointNames.size(); i++){
            try{
                cmd = feedback.getElementByName(jointNames[i]);
                if(!cmd.hasPosition()){ //Throw here, since we always need valid positions for this controller
                    LOG_ERROR("Element %s has no valid position value!", jointNames[i].c_str());
                    throw std::invalid_argument("Invalid position value");
                }
            }
            catch(std::exception e){
                LOG_ERROR("Feedback vector does not contain element %s", jointNames[i].c_str());
                throw e;
            }

            multiFieldCtrl->fields[i]->x.resize(1);
            multiFieldCtrl->fields[i]->x(0) = cmd.position;
        }
        return true;
    }
}

void JointRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &y){
    for(size_t i = 0; i < jointNames.size(); i++){
        controlOutput[i].speed = y(i);
    }
    controlOutput.time = base::Time::now();
    _controlOutput.write(controlOutput);
}

void JointRadialPotentialFields::setMaxInfluenceDistance(const base::VectorXd &distance){

    assert(controller != 0);

    // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
    // only do sth. if the size is not zero
    if(distance.size() == 0){

        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
        if(distance.size() != (int)multiFieldCtrl->fields.size()){
            LOG_ERROR("Max. Influence Distance vector must have same length as number of potential fields");
            throw std::invalid_argument("Invalid input");
        }
        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->dMax = distance(i);
    }
}

void JointRadialPotentialFields::setPotFieldCenters(const base::commands::Joints &centers){

    assert(controller != 0);

    MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
    if(centers.size() != multiFieldCtrl->fields.size()){
        LOG_ERROR("Pot. Field Center vector must have same length as number of potential fields");
        throw std::invalid_argument("Invalid input");
    }
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++){
        multiFieldCtrl->fields[i]->x0.resize(1);
        multiFieldCtrl->fields[i]->x0(0) = centers[i].position;
    }
}

void JointRadialPotentialFields::setOrder(const double order){

    assert(controller != 0);
    MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;
    for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
        multiFieldCtrl->fields[i]->M = order;
}
