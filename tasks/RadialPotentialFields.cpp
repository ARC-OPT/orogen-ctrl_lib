/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RadialPotentialFields.hpp"

using namespace ctrl_lib;

RadialPotentialFields::RadialPotentialFields(std::string const& name)
    : RadialPotentialFieldsBase(name)
{
}

RadialPotentialFields::RadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : RadialPotentialFieldsBase(name, engine)
{
}

RadialPotentialFields::~RadialPotentialFields()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RadialPotentialFields.hpp for more detailed
// documentation about them.

bool RadialPotentialFields::configureHook()
{
    if (! RadialPotentialFieldsBase::configureHook())
        return false;
    return true;
}
bool RadialPotentialFields::startHook()
{
    if (! RadialPotentialFieldsBase::startHook())
        return false;
    return true;
}
void RadialPotentialFields::updateHook()
{
    RadialPotentialFieldsBase::updateHook();
}
void RadialPotentialFields::errorHook()
{
    RadialPotentialFieldsBase::errorHook();
}
void RadialPotentialFields::stopHook()
{
    RadialPotentialFieldsBase::stopHook();
}
void RadialPotentialFields::cleanupHook()
{
    RadialPotentialFieldsBase::cleanupHook();
}
