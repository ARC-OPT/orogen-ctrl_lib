/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointLimitAvoidance.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>

using namespace ctrl_lib;

JointLimitAvoidance::JointLimitAvoidance(std::string const& name)
    : JointLimitAvoidanceBase(name)
{
}

JointLimitAvoidance::JointLimitAvoidance(std::string const& name, RTT::ExecutionEngine* engine)
    : JointLimitAvoidanceBase(name, engine)
{
}

JointLimitAvoidance::~JointLimitAvoidance()
{
}

bool JointLimitAvoidance::configureHook()
{
    if (! JointLimitAvoidanceBase::configureHook())
        return false;

    joint_limits = _joint_limits.get();

    if(joint_limits.size() != field_names.size()){
        LOG_ERROR("%s: Joint limit vector has size %i, but field names has size %i", this->getName().c_str(), joint_limits.size(), field_names.size());
        return false;
    }

    std::vector<PotentialField*> fields;
    for(size_t i = 0; i < field_names.size(); i++)
        fields.push_back(new RadialPotentialField(1, _order.get()));

    ((PotentialFieldsController*)controller)->setFields(fields);

    setInfluenceDistance(influence_distance);

    control_output.resize(field_names.size());
    control_output.names = field_names;

    return true;
}

bool JointLimitAvoidance::readSetpoint(){
    has_pot_field_centers = true;
    return true; //always return true here, since we configure the potential field centers
}

bool JointLimitAvoidance::readFeedback(){
    if(_feedback.read(feedback) == RTT::NewData)
        has_feedback = true;

    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

    if(has_feedback){
        extractPositions(feedback, field_names, position_raw);

        for(uint i = 0; i < position_raw.size(); i++){

            ctrl->fields[i]->pot_field_center.resize(1);

            if(position_raw(i) >= joint_limits[i].max.position)
                position_raw(i) = joint_limits[i].max.position - 1e-5;
            if(position_raw(i) <= joint_limits[i].min.position)
                position_raw(i) = joint_limits[i].min.position + 1e-5;

            if(fabs(joint_limits[i].max.position - position_raw(i))  < fabs(position_raw(i) - joint_limits[i].min.position))
                ctrl->fields[i]->pot_field_center(0) = joint_limits[i].max.position;
            else
                ctrl->fields[i]->pot_field_center(0) = joint_limits[i].min.position;

            ctrl->fields[i]->position.resize(1);
            ctrl->fields[i]->position(0) = position_raw(i);
        }

        _current_feedback.write(feedback);
        return true;
    }
    else
        return false;
}

void JointLimitAvoidance::writeControlOutput(const base::VectorXd& control_output_raw){
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = control_output_raw(i);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}

void JointLimitAvoidance::writeActivationFunction()
{
    activation.resize(field_names.size());

    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

    for(int i = 0; i < control_output_raw.size(); i++){
        double dist = fabs(ctrl->fields[i]->position(0) - ctrl->fields[i]->pot_field_center(0));
        activation(i) = fabs(dist - influence_distance(i)) / influence_distance(i);
    }
    _activation.write(activation_function.compute(activation));
}

bool JointLimitAvoidance::startHook()
{
    if (! JointLimitAvoidanceBase::startHook())
        return false;
    return true;
}
void JointLimitAvoidance::updateHook()
{
    JointLimitAvoidanceBase::updateHook();
}
void JointLimitAvoidance::errorHook()
{
    JointLimitAvoidanceBase::errorHook();
}
void JointLimitAvoidance::stopHook()
{
    JointLimitAvoidanceBase::stopHook();
}
void JointLimitAvoidance::cleanupHook()
{
    JointLimitAvoidanceBase::cleanupHook();
}
