/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CONTROLLERTASK_TASK_HPP
#define CTRL_LIB_CONTROLLERTASK_TASK_HPP

#include "ctrl_lib/ControllerTaskBase.hpp"

namespace ctrl_lib {

class Controller;

/*! \class ControllerTask Base task for all controllers */
class ControllerTask : public ControllerTaskBase
{
    friend class ControllerTaskBase;
protected:

    /** Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoint() = 0;
    /** Read all feedback values of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback() = 0;
    /** Compute output of the controller and write it to port. Returns control output */
    virtual void writeControlOutput(const base::VectorXd& control_output_raw) = 0;
    /** Write activation function to port*/
    virtual void writeActivationFunction() = 0;

    base::VectorXd control_output_raw, activation;
    ActivationFunction activation_function;
    Controller *controller;
    std::vector<std::string> field_names;

public:
    ControllerTask(std::string const& name = "ctrl_lib::ControllerTask");
    ControllerTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~ControllerTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

