/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CONTROLLERTASK_TASK_HPP
#define CTRL_LIB_CONTROLLERTASK_TASK_HPP

#include "ctrl_lib/ControllerTaskBase.hpp"
#include <ctrl_lib/ActivationFunction.hpp>
#include <base/Time.hpp>

namespace ctrl_lib {

/*!
* Base class for all controllers. Implements the controller state machine. Basic functionality is as follows:
*
*  1. Update Properties
*  2. Read Feedback term.
*  3. If a feedback term is available, read setpoint. Once there is a setpoint, control output will be written at all times.
*  3. Compute and write control output, depending on the implementation of the controller (derived task)
*  4. Compute activation. The activation indicates how much influence a control output has, compared to the other control outputs. Activation
*     values will be within 0..1. The activation ports can e.g. be connected to WBC in order to deactivate constraint variables and make unneeded
*     dof available for other tasks. A typical example is Joint limits avoidance. Usually one wants to activate the avoidance behavior only when
*     being close to the joint limits and not disturb other tasks when moving freely. Different activation functions can be chosen (e.g. linear, quadratic, ...)
*     in order to achieve smooth transitions.
*/
class ControllerTask : public ControllerTaskBase
{
    friend class ControllerTaskBase;
protected:
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback() = 0;
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint() = 0;
    /** Compute output of the controller*/
    virtual void updateController() = 0;
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function) = 0;

    std::vector<std::string> field_names;
    ActivationFunction activation_function;
    base::VectorXd tmp;
    base::Time stamp;

public:
    ControllerTask(std::string const& name = "ctrl_lib::ControllerTask");
    ControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~ControllerTask(){}
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

