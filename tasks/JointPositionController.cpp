/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPositionController.hpp"

using namespace ctrl_lib;

JointPositionController::JointPositionController(std::string const& name)
    : JointPositionControllerBase(name)
{
}

JointPositionController::JointPositionController(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPositionControllerBase(name, engine)
{
}

JointPositionController::~JointPositionController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See JointPositionController.hpp for more detailed
// documentation about them.

bool JointPositionController::configureHook()
{
    if (! JointPositionControllerBase::configureHook())
        return false;
    return true;
}
bool JointPositionController::startHook()
{
    if (! JointPositionControllerBase::startHook())
        return false;
    return true;
}
void JointPositionController::updateHook()
{
    JointPositionControllerBase::updateHook();
}
void JointPositionController::errorHook()
{
    JointPositionControllerBase::errorHook();
}
void JointPositionController::stopHook()
{
    JointPositionControllerBase::stopHook();
}
void JointPositionController::cleanupHook()
{
    JointPositionControllerBase::cleanupHook();
}
