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
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const base::VectorXd &ctrl_output_raw);
    /** Reset function. Implemented in derived task. This sets the control output to zero by setting setpoint and feedback to the same value.*/
    virtual void reset();

    void extractPositions(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& positions);
    void extractVelocities(const base::samples::Joints& joints, const std::vector<std::string> &names, base::VectorXd& velocities);

    base::commands::Joints setpoint, control_output;
    base::VectorXd setpoint_raw, feedback_raw, feedforward_raw;
    base::samples::Joints feedback;

public:
    JointPositionController(std::string const& name = "ctrl_lib::JointPositionController");
    JointPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPositionController();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

