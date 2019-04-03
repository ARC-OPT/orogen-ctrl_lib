/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointLimitAvoidance.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>
#include <base-logging/Logging.hpp>
#include <ctrl_lib/JointPotentialFieldsController.hpp>

using namespace ctrl_lib;

JointLimitAvoidance::JointLimitAvoidance(std::string const& name)
    : JointLimitAvoidanceBase(name){
}

JointLimitAvoidance::JointLimitAvoidance(std::string const& name, RTT::ExecutionEngine* engine)
    : JointLimitAvoidanceBase(name, engine){
}

bool JointLimitAvoidance::configureHook(){

    if(!JointLimitAvoidanceBase::configureHook())
        return false;

    controller = new JointLimitAvoidanceController(_joint_limits.get());
    controller->setPGain(_prop_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());
    if(joint_limits.size() != field_names.size()){
        LOG_ERROR("%s: Joint limit vector has to have same size as field names", this->getName().c_str());
        return false;
    }

    return true;
}

bool JointLimitAvoidance::startHook(){
    if (! CartesianPositionControllerBase::startHook())
        return false;
    _feedback.clear();
}

void JointLimitAvoidance::updateHook(){
    JointLimitAvoidanceBase::updateHook();
}

void JointLimitAvoidance::errorHook(){
    JointLimitAvoidanceBase::errorHook();
}

void JointLimitAvoidance::stopHook(){
    JointLimitAvoidanceBase::stopHook();
}

void JointLimitAvoidance::cleanupHook(){
    JointLimitAvoidanceBase::cleanupHook();
    delete controller;
}

bool JointLimitAvoidance::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else
        return true;
}

bool JointLimitAvoidance::readSetpoint(){
    return true;
}

const base::VectorXd& JointLimitAvoidance::updateController(){
    control_output = controller->update(feedback);

    _control_output.write(control_output);
    _field_infos.write(controller->getFieldInfos());
    _current_joint_limits.write(joint_limits);
}

const base::VectorXd& JointLimitAvoidance::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    tmp.setZero();

    const std::vector<PotentialFieldPtr> &fields = controller->getFields();
    for(uint i = 0; i < fields.size(); i++){
        double dist = fabs(fields[i]->position(0) - fields[i]->pot_field_center(0));
        if(dist > fields[i]->influence_distance)
            tmp(i) = 0;
        else
            tmp(i) = fabs(dist - fields[i]->influence_distance) / fields[i]->influence_distance;
    }

    return activation_function.compute(tmp);
}
