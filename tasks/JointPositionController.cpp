/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "JointPositionController.hpp"
#include <ctrl_lib/PositionControlFeedForward.hpp>

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
bool JointPositionController::configureHook()
{
    jointNames = _jointNames.get();

    x.resize(jointNames.size());
    v.resize(jointNames.size(), 0.0);
    xr.resize(jointNames.size());
    vr.resize(jointNames.size(), 0.0);
    ar.resize(jointNames.size(), 0.0);
    ctrlOutput.resize(jointNames.size());
    ctrlOutput.names = jointNames;

    controller = new PositionControlFeedForward(jointNames.size());

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

bool JointPositionController::readSetpoints(){
    if(_setpoint.read(setpoint) == RTT::NoData)
        return false;
    else{
        base::JointState cmd;
        for(size_t i = 0; i < jointNames.size(); i++)
        {
            cmd = setpoint.getElementByName(jointNames[i]);
            if(cmd.hasPosition())
                xr(i) = cmd.position;
            else{
                std::stringstream ss;
                ss << "Invalid Setpoint: Element " << i << " (" << jointNames[i] << ") has no valid position value";
                throw std::invalid_argument(ss.str());
            }
            if(cmd.hasSpeed())
                vr(i) = cmd.speed;
            if(cmd.hasAcceleration())
                ar(i) = cmd.acceleration;
        }
        return true;
    }
}

bool JointPositionController::readFeedback(){
    if(_feedback.read(feedback) == RTT::NoData)
        return false;
    else{
        base::JointState state;
        for(size_t i = 0; i < jointNames.size(); i++)
        {
            state = feedback.getElementByName(jointNames[i]);
            if(state.hasPosition())
                x(i) = state.position;
            else{
                std::stringstream ss;
                ss << "Invalid Feedback: Element " << i << " (" << jointNames[i] << ") has no valid position value";
                throw std::invalid_argument(ss.str());
            }
            if(state.hasSpeed())
                v(i) = state.speed;
        }
        return true;
    }
    return true;
}

void JointPositionController::sendControlOutput(){
    for(size_t i = 0; i < jointNames.size(); i++)
        ctrlOutput[i].speed = y(i);
    _ctrlOutput.write(ctrlOutput);
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
    delete controller;

    JointPositionControllerBase::cleanupHook();
}
