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

    jointNames = _jointNames.get();
    base::VectorXd maxInfluenceDistance = _maxInfluenceDistance.get();
    base::VectorXd potFieldCenters = _potFieldCenters.get();

    if(potFieldCenters.size() != jointNames.size()){
        LOG_ERROR("potFieldCenters vector must have same lenght as jointNames vector");
        return false;
    }
    // If maximum influence distance is not set, the default (inf) will be used in the potential field. So
    // also check here whether size is zero
    if(maxInfluenceDistance.size() != 0 && maxInfluenceDistance.size() != jointNames.size()){
        LOG_ERROR("maxInfluenceDistance vector must have same length as jointNames vector");
        return false;
    }

    controlOutput.resize(jointNames.size());
    controlOutput.names = jointNames;

    // Create a single 1-dimensional potential field per joint:
    std::vector<PotentialField*> multiFields;
    for(size_t i = 0; i < jointNames.size(); i++)
    {
        RadialRepulsivePotentialField* field = new RadialRepulsivePotentialField(1);
        if(maxInfluenceDistance.size() != 0) // Only set maxInfluenceDistance if a value has been configured, otherwise default (inf) will be used
            field->dMax = maxInfluenceDistance(i);
        field->x0.resize(1);
        field->x0(0) = potFieldCenters(i);
        field->M = _order.get();
        multiFields.push_back(field);
    }
    // The Controller contains all potential fields:
    controller = new MultiPotentialFields1D(multiFields, jointNames.size());

    if (! JointRadialPotentialFieldsBase::configureHook())
        return false;

    return true;
}

bool JointRadialPotentialFields::startHook(){
    if (! JointRadialPotentialFieldsBase::startHook())
        return false;
    return true;
}

bool JointRadialPotentialFields::readSetpoints(){
    if(_setpoint.readNewest(setpoint) == RTT::NewData){
        extractPositions(setpoint, jointNames, positions);
        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;

        for(size_t i = 0; i < jointNames.size(); i++){
            multiFieldCtrl->fields[i]->x0.resize(1);
            multiFieldCtrl->fields[i]->x0(0) = positions(i);
        }
    }
    return true; //Always return true here, since we don't necessarily need a setpoint (could be fixed by configuration)
}

bool JointRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        extractPositions(feedback, jointNames, positions);
        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;

        for(size_t i = 0; i < jointNames.size(); i++){
            multiFieldCtrl->fields[i]->x.resize(1);
            multiFieldCtrl->fields[i]->x(0) = positions(i);
        }
        return true;
    }
}

void JointRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &y){
    for(size_t i = 0; i < jointNames.size(); i++)
        controlOutput[i].speed = y(i);
    controlOutput.time = base::Time::now();
    _controlOutput.write(controlOutput);
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
    JointRadialPotentialFieldsBase::cleanupHook();
}
