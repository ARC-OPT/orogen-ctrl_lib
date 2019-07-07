/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_JOINTTORQUECONTROLLER_TASK_HPP
#define CTRL_LIB_JOINTTORQUECONTROLLER_TASK_HPP

#include "ctrl_lib/JointTorqueControllerBase.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib{

class JointTorquePIDController;

/*! \class JointTorqueController Joint Torque PID controller. Dimension of all fields has to be equal to field_names property*/
class JointTorqueController : public JointTorqueControllerBase
{
    friend class JointTorqueControllerBase;

public:
    JointTorqueController(std::string const& name = "ctrl_lib::JointTorqueController");
    JointTorqueController(std::string const& name, RTT::ExecutionEngine* engine);
    ~JointTorqueController(){}
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
    virtual void updateController();
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);

    base::commands::Joints setpoint, control_output, feedback;
    JointTorquePIDController* controller;

};
}

#endif

