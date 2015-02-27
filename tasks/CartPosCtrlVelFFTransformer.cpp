/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CartPosCtrlVelFFTransformer.hpp"
#include <base/Logging.hpp>


using namespace ctrl_lib;

CartPosCtrlVelFFTransformer::CartPosCtrlVelFFTransformer(std::string const& name)
    : CartPosCtrlVelFFTransformerBase(name)
{
}

CartPosCtrlVelFFTransformer::CartPosCtrlVelFFTransformer(std::string const& name, RTT::ExecutionEngine* engine)
    : CartPosCtrlVelFFTransformerBase(name, engine)
{
}

CartPosCtrlVelFFTransformer::~CartPosCtrlVelFFTransformer()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CartPosCtrlVelFFTransformer.hpp for more detailed
// documentation about them.

bool CartPosCtrlVelFFTransformer::configureHook()
{
    if (! CartPosCtrlVelFFTransformerBase::configureHook())
        return false;
    return true;
}
bool CartPosCtrlVelFFTransformer::startHook()
{
    if (! CartPosCtrlVelFFTransformerBase::startHook())
        return false;
    return true;
}
void CartPosCtrlVelFFTransformer::updateHook()
{
    CartPosCtrlVelFFTransformerBase::updateHook();


    //Get Transforms:
    if(!_setpoint2controlled_in.get(base::Time::now(), ref_)){

        if(state() != NO_SETPOINT_TRANSFORM)
            state(NO_SETPOINT_TRANSFORM);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No valid transformation available between %s and %s: No setpoint available.",
                      this->getName().c_str(), _controlled_in_frame.get().c_str(), _setpoint_frame.get().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    if(!_controlled_frame2controlled_in.get(base::Time::now(), cur_)){

        if(state() != NO_END_EFFECTOR_TRANSFORM)
            state(NO_END_EFFECTOR_TRANSFORM);

        if((base::Time::now() - stamp_).toSeconds() > 2){
            LOG_DEBUG("%s: No valid transformation available between %s and %s.",
                      this->getName().c_str(), _controlled_in_frame.get().c_str(), _controlled_frame_frame.get().c_str());
            stamp_ = base::Time::now();
        }
        return;
    }
    if(state() != RUNNING)
        state(RUNNING);

    control_step_and_write();
}
void CartPosCtrlVelFFTransformer::errorHook()
{
    CartPosCtrlVelFFTransformerBase::errorHook();
}
void CartPosCtrlVelFFTransformer::stopHook()
{
    CartPosCtrlVelFFTransformerBase::stopHook();
}
void CartPosCtrlVelFFTransformer::cleanupHook()
{
    CartPosCtrlVelFFTransformerBase::cleanupHook();
}
