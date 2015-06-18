/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianRadialPotentialFields.hpp"

using namespace ctrl_lib;

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name)
    : CartesianRadialPotentialFieldsBase(name)
{
}

CartesianRadialPotentialFields::CartesianRadialPotentialFields(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianRadialPotentialFieldsBase(name, engine)
{
}

CartesianRadialPotentialFields::~CartesianRadialPotentialFields()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CartesianRadialPotentialFields.hpp for more detailed
// documentation about them.

bool CartesianRadialPotentialFields::configureHook()
{
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
void CartesianRadialPotentialFields::updateHook()
{
    CartesianRadialPotentialFieldsBase::updateHook();
}
void CartesianRadialPotentialFields::errorHook()
{
    CartesianRadialPotentialFieldsBase::errorHook();
}
void CartesianRadialPotentialFields::stopHook()
{
    CartesianRadialPotentialFieldsBase::stopHook();
}
void CartesianRadialPotentialFields::cleanupHook()
{
    CartesianRadialPotentialFieldsBase::cleanupHook();
}
