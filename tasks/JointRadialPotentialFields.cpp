/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointRadialPotentialFields.hpp"

using namespace ctrl_lib;

JointRadialPotentialFields::JointRadialPotentialFields(std::string const& name)
    : JointRadialPotentialFieldsBase(name)
{
}

JointRadialPotentialFields::JointRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : JointRadialPotentialFieldsBase(name, engine)
{
}

JointRadialPotentialFields::~JointRadialPotentialFields()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See JointRadialPotentialFields.hpp for more detailed
// documentation about them.

bool JointRadialPotentialFields::configureHook()
{
    if (! JointRadialPotentialFieldsBase::configureHook())
        return false;
    return true;
}
bool JointRadialPotentialFields::startHook()
{
    if (! JointRadialPotentialFieldsBase::startHook())
        return false;
    return true;
}
void JointRadialPotentialFields::updateHook()
{
    JointRadialPotentialFieldsBase::updateHook();
}
void JointRadialPotentialFields::errorHook()
{
    JointRadialPotentialFieldsBase::errorHook();
}
void JointRadialPotentialFields::stopHook()
{
    JointRadialPotentialFieldsBase::stopHook();
}
void JointRadialPotentialFields::cleanupHook()
{
    JointRadialPotentialFieldsBase::cleanupHook();
}
