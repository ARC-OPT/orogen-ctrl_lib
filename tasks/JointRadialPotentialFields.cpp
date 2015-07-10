/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointRadialPotentialFields.hpp"
#include <ctrl_lib/RadialRepulsivePotentialField.hpp>
#include <ctrl_lib/MultiPotentialFields.hpp>

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
    std::vector<double> potFieldCenters = _potFieldCenters.get();

    assert(jointNames.size() == maxInfluenceDistance.size());
    assert(jointNames.size() == potFieldCenters.size());

    std::vector<PotentialField*> multiFields;
    for(size_t i = 0; i < jointNames.size(); i++)
    {
        assert(potFieldCenters[i].size() == 1);
        RadialRepulsivePotentialField* field = new RadialRepulsivePotentialField(jointNames.size());
        field->dMax = maxInfluenceDistance(i);
        field->x0 = potFieldCenters[i];
        field->M = _order.get();
        multiFields.push_back(field);
    }
    controller = new MultiPotentialFields(multiFields, jointNames.size());

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

}

bool JointRadialPotentialFields::readFeedback(){

}

void JointRadialPotentialFields::writeControlOutput(const Eigen::VectorXd &y){

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
    JointRadialPotentialFieldsBase::cleanupHook();
}
