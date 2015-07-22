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

    std::vector<base::VectorXd> potFieldCenters = _potFieldCenters.get();
    Eigen::VectorXd maxInfluenceDistance = _maxInfluenceDistance.get();

    // If maximum influence distance is not set, the default (inf) will be used in the potential field. So,
    // also check whether size is zero here
    if(maxInfluenceDistance.size() != 0 && maxInfluenceDistance.size() != potFieldCenters.size()){
        LOG_ERROR("maxInfluenceDistance vector must have same length as potFieldCenters vector");
        return false;
    }

    //Create potential fields. All fields will have dimension 3 here!
    std::vector<PotentialField*> multiFields;
    for(uint i = 0; i < potFieldCenters.size(); i++)
    {
        if(potFieldCenters[i].size() != 3){
            LOG_ERROR("Dimension of all potential fields must be 3 for this controller");
            return false;
        }
        RadialRepulsivePotentialField* field = new RadialRepulsivePotentialField(3);

        if(maxInfluenceDistance.size() != 0)  // Only set maxInfluenceDistance if a value has been configured, otherwise default (inf) will be used
            field->dMax = maxInfluenceDistance(i);
        field->x0 = potFieldCenters[i];
        field->M = _order.get();
        multiFields.push_back(field);
    }
    // The Controller contains all potential fields:
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
    if(_setpoint.readNewest(setpoint) == RTT::NewData){

        MultiPotentialFields* multiFieldCtrl = (MultiPotentialFields*)controller;
        if(setpoint.size() != multiFieldCtrl->fields.size()){
            LOG_ERROR("Size of setpoint vector is %i, but this controller has %i potential fields", setpoint.size(), multiFieldCtrl->fields.size());
            return false;
        }

        for(size_t i = 0; i < multiFieldCtrl->fields.size(); i++)
            multiFieldCtrl->fields[i]->x0 = setpoint[i].position;
    }
    return true; //Always return true here, since we don't necessarily need a setpoint (could be fixed by configuration)
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
