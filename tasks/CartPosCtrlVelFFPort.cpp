/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPosCtrlVelFFPort.hpp"
#include <base/Logging.hpp>

using namespace ctrl_lib;

CartPosCtrlVelFFPort::CartPosCtrlVelFFPort(std::string const& name)
    : CartPosCtrlVelFFPortBase(name)
{
}

CartPosCtrlVelFFPort::CartPosCtrlVelFFPort(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPosCtrlVelFFPortBase(name, engine)
{
}

CartPosCtrlVelFFPort::~CartPosCtrlVelFFPort()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CartPosCtrlVelFFPort.hpp for more detailed
// documentation about them.

bool CartPosCtrlVelFFPort::configureHook()
{
    if (! CartPosCtrlVelFFPortBase::configureHook())
        return false;
    return true;
}
bool CartPosCtrlVelFFPort::startHook()
{
    if (! CartPosCtrlVelFFPortBase::startHook())
        return false;
    return true;
}
void CartPosCtrlVelFFPort::updateHook()
{
    CartPosCtrlVelFFPortBase::updateHook();

    //Get Transforms:
    RTT::FlowStatus st;
    st = _setpoint.readNewest(ref_);
    if(st == RTT::NoData)
        return;

    st = _status.readNewest(cur_);
    if(st == RTT::NoData)
        return;

    if(st == RTT::OldData){
        if(state() != NO_END_EFFECTOR_TRANSFORM)
            state(NO_END_EFFECTOR_TRANSFORM);

        LOG_DEBUG("%s: No status update received since %.3f seconds",
                  this->getName().c_str(), (base::Time::now() - stamp_).toSeconds());
        return;
    }
    else{
        stamp_ = base::Time::now();
        if(state() != RUNNING)
            state(RUNNING);
    }

    control_step_and_write();
}
void CartPosCtrlVelFFPort::errorHook()
{
    CartPosCtrlVelFFPortBase::errorHook();
}
void CartPosCtrlVelFFPort::stopHook()
{
    CartPosCtrlVelFFPortBase::stopHook();
}
void CartPosCtrlVelFFPort::cleanupHook()
{
    CartPosCtrlVelFFPortBase::cleanupHook();
}
