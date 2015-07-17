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

    assert(jointNames.size() == maxInfluenceDistance.size());
    assert(jointNames.size() == potFieldCenters.size());

    std::vector<PotentialField*> multiFields;
    for(size_t i = 0; i < jointNames.size(); i++)
    {
        assert(potFieldCenters[i].size() == 1);
        RadialRepulsivePotentialField* field = new RadialRepulsivePotentialField(jointNames.size());
        field->dMax = maxInfluenceDistance(i);
        field->x0.resize(1);
        field->x0(0) = potFieldCenters(i);
        field->M = _order.get();
        multiFields.push_back(field);
    }
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
    if(_setpoint.read(setpoint) == RTT::NoData)
        return false;
    else{
        extractPositions(setpoint, jointNames, positions);
        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;

        for(size_t i = 0; i < jointNames.size(); i++)
            multiFieldCtrl->fields[i]->x0(0) = positions(i);
        return true;
    }
}

bool JointRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        extractPositions(feedback, jointNames, positions);
        MultiPotentialFields1D* multiFieldCtrl = (MultiPotentialFields1D*)controller;

        for(size_t i = 0; i < jointNames.size(); i++)
            multiFieldCtrl->fields[i]->x(0) = positions(i);
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
