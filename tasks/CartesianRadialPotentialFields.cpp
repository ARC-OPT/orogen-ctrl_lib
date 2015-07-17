/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"
#include <ctrl_lib/MultiPotentialFields.hpp>
#include <ctrl_lib/RadialRepulsivePotentialField.hpp>

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

    std::vector<base::VectorXd> potFieldCenters = _potFieldCenters.get();
    Eigen::VectorXd maxInfluenceDistance = _maxInfluenceDistance.get();
    assert(maxInfluenceDistance.size() == potFieldCenters.size());

    std::vector<PotentialField*> multiFields;
    for(uint i = 0; i < potFieldCenters.size(); i++)
    {
        assert(potFieldCenters[i].size() == 3);
        RadialRepulsivePotentialField* field = new RadialRepulsivePotentialField(3);
        field->dMax = maxInfluenceDistance(i);
        field->x0 = potFieldCenters[i];
        field->M = _order.get();
        multiFields.push_back(field);
    }
    controller = new MultiPotentialFields(multiFields, 3);

    if (! CartesianRadialPotentialFieldsBase::configureHook())
        return false;
    return true;
}

bool CartesianRadialPotentialFields::startHook()
{
    if (! CartesianRadialPotentialFieldsBase::startHook())
        return false;
    return true;
}

bool CartesianRadialPotentialFields::readSetpoints(){
    if(_setpoint.read(setpoint) == RTT::NewData){
        MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
        assert(setpoint.size() == multiFieldCtrl->fields.size());
        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->x0 = setpoint[i].position;
    }
    return true;
}

bool CartesianRadialPotentialFields::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->x = feedback.position;
        return true;
    }
}

void CartesianRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &y){
    for(uint i = 0; i < 3; i++)
        controlOutput.velocity(i) = y(i);
    controlOutput.time = base::Time::now();
    _controlOutput.write(controlOutput);
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
    CartesianRadialPotentialFieldsBase::cleanupHook();
}
