/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/JointPositionControllerBase.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib {

/*! \class JointPositionController Implementation of PositionControlFeedForward in joint space. See ctrl_lib/PositionControlFeedForward.hpp for details*/
class JointPositionController : public JointPositionControllerBase
{
    friend class JointPositionControllerBase;
protected:

    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw);
    /** Reset setpoint to actual value. Control output will be approx. zero after that action*/
    virtual void reset();
    /** Clear setpoint. No control output will be written after that*/
    virtual void clearSetpoint();

    void extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& positions);
    void extractVelocities(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& velocities);

    base::commands::Joints setpoint, control_output;
    base::VectorXd setpoint_raw, feedback_raw, feedforward_raw;
    base::samples::Joints feedback;

public:
    JointPositionController(std::string const& name = "ctrl_lib::JointPositionController");
    JointPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPositionController(){}
    bool configureHook(){return JointPositionControllerBase::configureHook();}
    bool startHook(){return JointPositionControllerBase::startHook();}
    void updateHook(){JointPositionControllerBase::updateHook();}
    void errorHook(){JointPositionControllerBase::errorHook();}
    void stopHook(){JointPositionControllerBase::stopHook();}
    void cleanupHook(){JointPositionControllerBase::cleanupHook();}
};
}

#endif

