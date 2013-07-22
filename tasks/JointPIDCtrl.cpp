/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPIDCtrl.hpp"

using namespace ctrl_lib;

JointPIDCtrl::JointPIDCtrl(std::string const& name)
    : JointPIDCtrlBase(name)
{
}

JointPIDCtrl::JointPIDCtrl(std::string const& name, RTT::ExecutionEngine* engine)
    : JointPIDCtrlBase(name, engine)
{
}

JointPIDCtrl::~JointPIDCtrl()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See JointPIDCtrl.hpp for more detailed
// documentation about them.

bool JointPIDCtrl::configureHook()
{
    if (! JointPIDCtrlBase::configureHook())
        return false;
    return true;
}
bool JointPIDCtrl::startHook()
{
    if (! JointPIDCtrlBase::startHook())
        return false;
    return true;
}
void JointPIDCtrl::updateHook()
{
    JointPIDCtrlBase::updateHook();
}
void JointPIDCtrl::errorHook()
{
    JointPIDCtrlBase::errorHook();
}
void JointPIDCtrl::stopHook()
{
    JointPIDCtrlBase::stopHook();
}
void JointPIDCtrl::cleanupHook()
{
    JointPIDCtrlBase::cleanupHook();
}
