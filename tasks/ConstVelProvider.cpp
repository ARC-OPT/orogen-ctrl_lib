/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ConstVelProvider.hpp"

using namespace ctrl_lib;

ConstVelProvider::ConstVelProvider(std::string const& name)
    : ConstVelProviderBase(name)
{
}

ConstVelProvider::ConstVelProvider(std::string const& name, RTT::ExecutionEngine* engine)
    : ConstVelProviderBase(name, engine)
{
}

ConstVelProvider::~ConstVelProvider()
{
}

bool ConstVelProvider::configureHook()
{
    if (! ConstVelProviderBase::configureHook())
        return false;

    control_output.sourceFrame = _source_frame.get();
    control_output.targetFrame = _target_frame.get();
    control_output.velocity = _velocity.get();
    control_output.angular_velocity = _angular_velocity.get();

    return true;
}
bool ConstVelProvider::startHook()
{
    if (! ConstVelProviderBase::startHook())
        return false;
    return true;
}
void ConstVelProvider::updateHook()
{
    ConstVelProviderBase::updateHook();

    control_output.time = base::Time::now();
    _control_output.write(control_output);
}
void ConstVelProvider::errorHook()
{
    ConstVelProviderBase::errorHook();
}
void ConstVelProvider::stopHook()
{
    ConstVelProviderBase::stopHook();
}
void ConstVelProvider::cleanupHook()
{
    ConstVelProviderBase::cleanupHook();
}
