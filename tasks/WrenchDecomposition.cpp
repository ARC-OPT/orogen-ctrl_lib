/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WrenchDecomposition.hpp"

using namespace ctrl_lib;

WrenchDecomposition::WrenchDecomposition(std::string const& name)
    : WrenchDecompositionBase(name)
{
}

WrenchDecomposition::WrenchDecomposition(std::string const& name, RTT::ExecutionEngine* engine)
    : WrenchDecompositionBase(name, engine)
{
}

WrenchDecomposition::~WrenchDecomposition()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WrenchDecomposition.hpp for more detailed
// documentation about them.

bool WrenchDecomposition::configureHook()
{
    if (! WrenchDecompositionBase::configureHook())
        return false;
    return true;
}
bool WrenchDecomposition::startHook()
{
    if (! WrenchDecompositionBase::startHook())
        return false;
    return true;
}
void WrenchDecomposition::updateHook()
{
    WrenchDecompositionBase::updateHook();
}
void WrenchDecomposition::errorHook()
{
    WrenchDecompositionBase::errorHook();
}
void WrenchDecomposition::stopHook()
{
    WrenchDecompositionBase::stopHook();
}
void WrenchDecomposition::cleanupHook()
{
    WrenchDecompositionBase::cleanupHook();
}
