/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointLimitAvoidance.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>
#include <base/Logging.hpp>
#include <ctrl_lib/JointPotentialFieldsController.hpp>

using namespace ctrl_lib;

JointLimitAvoidance::JointLimitAvoidance(std::string const& name)
    : JointLimitAvoidanceBase(name){
}

JointLimitAvoidance::JointLimitAvoidance(std::string const& name, RTT::ExecutionEngine* engine)
    : JointLimitAvoidanceBase(name, engine){
}

bool JointLimitAvoidance::configureHook(){
    controller = new JointPotentialFieldsController(_field_names.get().size());
    if(!JointLimitAvoidanceBase::configureHook())
        return false;
    return true;
}

bool JointLimitAvoidance::readSetpoint(){
    if(!controller->hasPotFieldCenters()){
        joint_limits = _joint_limits.get();

        if(joint_limits.size() != field_names.size())
            throw std::invalid_argument("Joint limit vector has to have same size as field names");

        std::vector<PotentialField*> fields;
        for(size_t i = 0; i < field_names.size(); i++)
            fields.push_back(new RadialPotentialField(1));
        controller->setFields(fields);
        controller->setInfluenceDistance(_influence_distance.get());
    }

    return controller->hasPotFieldCenters();
}

bool JointLimitAvoidance::readFeedback(){

    if(_feedback.read(feedback) == RTT::NewData){

        extractPositions(feedback, field_names, position_raw);

        for(uint i = 0; i < position_raw.size(); i++){
            // Prevent infinite control action:
            if(position_raw(i) >= joint_limits[i].max.position)
                position_raw(i) = joint_limits[i].max.position - 1e-5;
            if(position_raw(i) <= joint_limits[i].min.position)
                position_raw(i) = joint_limits[i].min.position + 1e-5;

            controller->setPosition(position_raw);

            base::VectorXd center(1);
            if(fabs(joint_limits[i].max.position - position_raw(i))  < fabs(position_raw(i) - joint_limits[i].min.position))
                center(0) = joint_limits[i].max.position;
            else
                center(0) = joint_limits[i].min.position;
            controller->setPotFieldCenters(i, center);
        }

        _current_feedback.write(feedback);
    }

    return controller->hasPosition();
}

void JointLimitAvoidance::writeControlOutput(const base::VectorXd& control_output_raw){
    control_output.resize(field_names.size());
    control_output.names = field_names;
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = control_output_raw(i);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

const base::VectorXd& JointLimitAvoidance::computeActivation(ActivationFunction &activation_function){
    tmp.resize(controller->getDimension());
    tmp.setZero();

    const std::vector<PotentialField*> &fields = controller->getFields();
    for(uint i = 0; i < fields.size(); i++){
        double dist = fabs(fields[i]->position(0) - fields[i]->pot_field_center(0));
        if(dist > fields[i]->influence_distance)
            tmp(i) = 0;
        else
            tmp(i) = fabs(dist - fields[i]->influence_distance) / fields[i]->influence_distance;
    }

    return activation_function.compute(tmp);
}

void JointLimitAvoidance::extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& positions){
    positions.resize(names.size());
    for(size_t i = 0; i < names.size(); i++){
        const base::JointState& elem = joints.getElementByName(names[i]);
        if(!elem.hasPosition()){
            LOG_ERROR("%s: Element %s does not have a valid position value", this->getName().c_str(), names[i].c_str());
            throw std::invalid_argument("Invalid joints vector");
        }
        positions(i) = elem.position;
    }
}
