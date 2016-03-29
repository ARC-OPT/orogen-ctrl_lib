/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_PROPORTIONALCONTROLLERTASK_TASK_HPP
#define CTRL_LIB_PROPORTIONALCONTROLLERTASK_TASK_HPP

#include "ctrl_lib/ProportionalControllerTaskBase.hpp"
#include <ctrl_lib/ProportionalFeedForwardController.hpp>
#include <ctrl_lib/ActivationFunction.hpp>

namespace ctrl_lib {

class ProportionalControllerTask : public ProportionalControllerTaskBase
{
    friend class ProportionalControllerTaskBase;
protected:

    /** Update all available (dynamic) controller properties*/
    virtual void updateControllerProperties();
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback() = 0;
    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint() = 0;
    /** Compute output of the controller*/
    virtual const base::VectorXd& updateController();
    /** Write control output to port*/
    virtual void writeControlOutput(const base::VectorXd& control_output_raw) = 0;
    /** Compute Activation function*/
    virtual const base::VectorXd& computeActivation(ActivationFunction& activation_function);

    ProportionalFeedForwardController* controller;
    base::VectorXd tmp;

public:
    ProportionalControllerTask(std::string const& name = "ctrl_lib::ProportionalControllerTask");
    ProportionalControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~ProportionalControllerTask(){}
    bool configureHook();
    bool startHook();
    void updateHook(){ProportionalControllerTaskBase::updateHook();}
    void errorHook(){ProportionalControllerTaskBase::errorHook();}
    void stopHook(){ProportionalControllerTaskBase::stopHook();}
    void cleanupHook();
};
}

#endif

