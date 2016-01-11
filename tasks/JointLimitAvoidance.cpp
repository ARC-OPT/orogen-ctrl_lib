/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointLimitAvoidance.hpp"
#include <ctrl_lib/RadialPotentialField.hpp>
#include <base/Logging.hpp>
#include <ctrl_lib/PotentialFieldsController.hpp>

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
    controller = new PotentialFieldsController(_field_names.get().size());

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
    ((PotentialFieldsController*)controller)->setInfluenceDistance(_influence_distance.get());

    control_output.resize(field_names.size());
    control_output.names = field_names;

    return true;
}

bool JointLimitAvoidance::readSetpoint(){
    return true; //always return true here, since we configure the potential field centers by setting the joint limits
}

bool JointLimitAvoidance::readFeedback(){

    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;

    if(_feedback.read(feedback) == RTT::NewData){


        extractPositions(feedback, field_names, position_raw);

        for(uint i = 0; i < position_raw.size(); i++){
            // Prevent infinite control action:
            if(position_raw(i) >= joint_limits[i].max.position)
                position_raw(i) = joint_limits[i].max.position - 1e-5;
            if(position_raw(i) <= joint_limits[i].min.position)
                position_raw(i) = joint_limits[i].min.position + 1e-5;
        }

        ctrl->setFeedback(position_raw);

        for(uint i = 0; i < position_raw.size(); i++){
            base::VectorXd center(1);
            if(fabs(joint_limits[i].max.position - position_raw(i))  < fabs(position_raw(i) - joint_limits[i].min.position))
                center(0) = joint_limits[i].max.position;
            else
                center(0) = joint_limits[i].min.position;
            ctrl->setPotFieldCenters(i, center);
        }

        _current_feedback.write(feedback);
    }

    return ctrl->hasFeedback();
}

void JointLimitAvoidance::writeControlOutput(const base::VectorXd& control_output_raw){
    for(size_t i = 0; i < field_names.size(); i++)
        control_output[i].speed = control_output_raw(i);

    control_output.time = base::Time::now();
    _control_output.write(control_output);
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

bool JointLimitAvoidance::startHook(){
    if (! JointLimitAvoidanceBase::startHook())
        return false;
    return true;
}

void JointLimitAvoidance::updateHook(){
    PotentialFieldsController* ctrl = (PotentialFieldsController*)controller;
    ctrl->setInfluenceDistance(_influence_distance.get());

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
}
