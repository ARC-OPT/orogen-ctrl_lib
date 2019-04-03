/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP
#define CTRL_LIB_JOINTPOSITIONCONTROLLER_TASK_HPP

#include "ctrl_lib/JointPositionControllerBase.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib {

class JointPosPDController;

/*! \class JointPositionController Implementation of PositionControlFeedForward in joint space. See ctrl_lib/PositionControlFeedForward.hpp for details*/
class JointPositionController : public JointPositionControllerBase
{
    friend class JointPositionControllerBase;

public:
    JointPositionController(std::string const& name = "ctrl_lib::JointPositionController");
    JointPositionController(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointPositionController(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();

protected:
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback();
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint();
    /** Compute output of the controller*/
    virtual const base::VectorXd& updateController();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw){}
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);

    base::commands::Joints setpoint, control_output, feedback;
    JointPosPDController* controller;
};
}

#endif

