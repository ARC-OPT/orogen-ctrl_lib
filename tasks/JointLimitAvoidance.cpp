/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointLimitAvoidance.hpp"
#include <wbc/controllers/RadialPotentialField.hpp>
#include <base-logging/Logging.hpp>
#include <wbc/controllers/JointLimitAvoidanceController.hpp>

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

    joint_limits = _joint_limits.get();
    joint_limits.names = field_names;
    controller = new JointLimitAvoidanceController(joint_limits, _influence_distance.get());
    controller->setPGain(_p_gain.get());
    controller->setMaxControlOutput(_max_control_output.get());

    return true;
}

bool JointLimitAvoidance::startHook(){
    if (! JointLimitAvoidanceBase::startHook())
        return false;
    _feedback.clear();
    return true;
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
    if(_feedback.readNewest(feedback) == RTT::NoData)
        return false;
    else{
        _current_feedback.write(feedback);
        return true;
    }
}

bool JointLimitAvoidance::readSetpoint(){
    return true;
}

void JointLimitAvoidance::updateController(){
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
        double dist = fields[i]->distance.norm();
        if(dist > fields[i]->influence_distance)
            tmp(i) = 0;
        else
            tmp(i) = fabs(dist - fields[i]->influence_distance) / fields[i]->influence_distance;
    }

    return activation_function.compute(tmp);
}
