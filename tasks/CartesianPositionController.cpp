/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartesianPositionController.hpp"

using namespace ctrl_lib;

CartesianPositionController::CartesianPositionController(std::string const& name)
    : CartesianPositionControllerBase(name)
{
}

CartesianPositionController::CartesianPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : CartesianPositionControllerBase(name, engine)
{
}

CartesianPositionController::~CartesianPositionController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CartesianPositionController.hpp for more detailed
// documentation about them.

bool CartesianPositionController::configureHook()
{
    if (! CartesianPositionControllerBase::configureHook())
        return false;
    return true;
}
bool CartesianPositionController::startHook()
{
    if (! CartesianPositionControllerBase::startHook())
        return false;
    return true;
}
void CartesianPositionController::updateHook()
{
    CartesianPositionControllerBase::updateHook();
}
void CartesianPositionController::errorHook()
{
    CartesianPositionControllerBase::errorHook();
}
void CartesianPositionController::stopHook()
{
    CartesianPositionControllerBase::stopHook();
}
void CartesianPositionController::cleanupHook()
{
    CartesianPositionControllerBase::cleanupHook();
}
