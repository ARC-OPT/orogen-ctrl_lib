/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPIDCtrl.hpp"

using namespace ctrl_lib;

CartPIDCtrl::CartPIDCtrl(std::string const& name)
    : CartPIDCtrlBase(name)
{
}

CartPIDCtrl::CartPIDCtrl(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPIDCtrlBase(name, engine)
{
}

CartPIDCtrl::~CartPIDCtrl()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CartPIDCtrl.hpp for more detailed
// documentation about them.

bool CartPIDCtrl::configureHook()
{
    if (! CartPIDCtrlBase::configureHook())
        return false;
    return true;
}
bool CartPIDCtrl::startHook()
{
    if (! CartPIDCtrlBase::startHook())
        return false;
    return true;
}
void CartPIDCtrl::updateHook()
{
    CartPIDCtrlBase::updateHook();
}
void CartPIDCtrl::errorHook()
{
    CartPIDCtrlBase::errorHook();
}
void CartPIDCtrl::stopHook()
{
    CartPIDCtrlBase::stopHook();
}
void CartPIDCtrl::cleanupHook()
{
    CartPIDCtrlBase::cleanupHook();
}
