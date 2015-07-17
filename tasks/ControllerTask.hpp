/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef CTRL_LIB_CONTROLLERTASK_TASK_HPP
#define CTRL_LIB_CONTROLLERTASK_TASK_HPP

#include "ctrl_lib/ControllerTaskBase.hpp"

namespace ctrl_lib {

class Controller;

/*! \class ControllerTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     *
Base task for all controllers

     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','ctrl_lib::ControllerTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
class ControllerTask : public ControllerTaskBase
{
    friend class ControllerTaskBase;
protected:

    Controller* controller;

    /** Implement in base class. Read all setpoints of the controller. Return false if there is no setpoint, true otherwise */
    virtual bool readSetpoints() = 0;
    /** Implement in base class. Read all feedback value of the controller. Return false if there is no feedback, true otherwise */
    virtual bool readFeedback() = 0;
    /** Write the output of the controller to a port */
    virtual void writeControlOutput(const Eigen::VectorXd &y) = 0;

    Eigen::VectorXd y; /** Control output */

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
